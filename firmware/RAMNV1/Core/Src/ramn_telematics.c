/*
 * ramn_telematics.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 TOYOTA MOTOR CORPORATION.
 * ALL RIGHTS RESERVED.</center></h2>
 *
 * This software component is licensed by TOYOTA MOTOR CORPORATION under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "ramn_telematics.h"
#include "ramn_config.h"
#include "ramn_canfd.h"
#include "ramn_uart.h"
#include <stdio.h>

// Define to enable UART debug output for SPI↔ESP32 communication.
// Comment out to disable.
#define TELEMATICS_SPI_DEBUG

// RAMN_UART_SendFromTask is where this output goes, and ENABLE_UART is defined
// for TARGET_ECUD only (see ramn_config.h). This module is compiled for every target, including on ECU A, which
// has no UART: the function is neither declared nor compiled there, so leaving
// the flag on makes every debug call an implicit declaration that fails at
// link. Turn the flag off where there is nothing to print to, rather than
// wrapping each call site.
#if defined(TELEMATICS_SPI_DEBUG) && !defined(ENABLE_UART)
#undef TELEMATICS_SPI_DEBUG
#endif

// Define to enable UART debug output for all outgoing CAN messages.
// Comment out to disable.
//#define TELEMATICS_CAN_DEBUG
#if defined(TELEMATICS_CAN_DEBUG) && !defined(ENABLE_UART)
#undef TELEMATICS_CAN_DEBUG   // same reason as above; off today, guarded anyway
#endif

// External reference to CAN TX queue
extern StreamBufferHandle_t CANTxDataStreamBufferHandle;

// ============================================================================
// DOUBLE-BUFFER ARCHITECTURE FOR CAN-TO-SPI BRIDGE WITH DMA
// ============================================================================
// Strategy: Use two buffers that swap atomically
// - Active buffer: CAN RX task writes here (short critical section ~5µs)
// - Flush buffer: SPI DMA reads from here (no CPU blocking)
// - Swap operation: Atomic pointer exchange with 1µs critical section
//
// Benefits:
// - Minimal critical sections (5µs for write, 1µs for swap vs 100ms old blocking)
// - Non-blocking DMA-based SPI transmission (zero CPU time during transfer)
// - No buffer copying or shifting (direct pointer swap)
// - Excellent real-time performance (20,000x improvement over blocking SPI)
// ============================================================================

#define SPI_TX_BUFFER_SIZE 1024  // Each buffer: ~10 CAN-FD or ~40 standard CAN messages

// Double buffers for TX (CAN → ESP32)
static uint8_t spiTxBufferA[SPI_TX_BUFFER_SIZE];
static uint8_t spiTxBufferB[SPI_TX_BUFFER_SIZE];

// Active buffer (CAN RX writes here - lock-free!)
static uint8_t* volatile activeBuffer = spiTxBufferA;
static volatile uint16_t activeBufferPos = 0;

// Flush buffer (SPI reads from here - lock-free!)
static uint8_t* volatile flushBuffer = spiTxBufferB;
static volatile uint16_t flushBufferSize = 0;
volatile RAMN_Bool_t spiTransmitBusy = False;  // Non-static: accessed from ramn_spi.c

// ===== NEW SPI OWNER LOCK =====
typedef enum {
    SPI_OWNER_NONE = 0,
    SPI_OWNER_TX,
    SPI_OWNER_POLL
} SPI_Owner_t;

static volatile SPI_Owner_t spiOwner = SPI_OWNER_NONE;  // Tracks who owns SPI peripheral

// Timing and thresholds
static volatile uint32_t spiTxLastFlushTick = 0;  // volatile: accessed from multiple tasks
#define SPI_FLUSH_INTERVAL_MS 10
#define SPI_BUFFER_FLUSH_THRESHOLD 896  // Flush when 128 bytes remain

// Stats printing interval
#define SPI_STATS_PRINT_INTERVAL_MS 1000  // Print stats every 1 second
static volatile uint32_t spiStatsLastPrintTick = 0;

// ============================================================================
// BIDIRECTIONAL SPI: ESP32 POLLING STATE MACHINE
// ============================================================================
#define SPI_POLL_INTERVAL_MS 50   // Normal poll interval (ms) — reduced to 1 ms during streaming
#define SPI_POLL_TIMEOUT_MS 10    // Max wait for ESP32 response
#define SPI_RX_BUFFER_SIZE 160    // Two packed messages per poll: 2×71 = 142 bytes + 18 padding

// Poll state machine
typedef enum {
	SPI_POLL_IDLE,              // Not polling
	SPI_POLL_REQUESTED,         // Poll request sent, waiting for response
	SPI_POLL_COMPLETE,          // Response received, ready to process
	SPI_POLL_TIMEOUT            // Response timeout, skip this poll
} SPI_PollState_t;

static volatile SPI_PollState_t spiPollState = SPI_POLL_IDLE;
static volatile uint32_t spiPollRequestTick = 0;
static volatile uint32_t spiPollCompleteTick = 0;  // Track when state changed to COMPLETE
static volatile uint32_t spiLastPollTick = 0;
static volatile uint32_t spiLastPollAttemptTick = 0;  // Track last attempt (even if failed due to busy)

// ============================================================================
// IMAGE STREAMING STATE MACHINE
// ============================================================================
typedef enum { STREAM_IDLE, KEYFRAME_ACTIVE, KEYFRAME_SENT, DELTA_ACTIVE } StreamState_t;
static StreamState_t streamState = STREAM_IDLE;

// Keyframe tracking
static uint16_t kfTotalChunks    = 0;
static uint16_t kfChunksSent     = 0;
static uint8_t  kfXOffset        = 0;
static uint8_t  kfYOffset        = 0;
static uint32_t kfAckWaitTick    = 0;
#define KF_ACK_TIMEOUT_MS 2000U

// Delta tracking
static uint8_t     deltaFrameSeq          = 0;
static uint8_t     deltaTileCount         = 0;
static RAMN_Bool_t deltaFirstTileOfFrame  = False;
static uint32_t    lastDeltaActivityTick  = 0;
#define DELTA_IDLE_TIMEOUT_MS 2000U

// ACK received from ECU A (set by RAMN_TELEMATICS_ProcessImageACK, read by Update)
static volatile RAMN_Bool_t kfAckReceived = False;
static volatile uint8_t     kfAckStatus   = 0x00U;

// UART scratch buffers.
//
// These are static, not locals, because every one of them is written on the
// periodic task, whose ENTIRE stack is 1 KB (RAMN_PeriodicBuffer[256] in
// main.c), down a chain whose frames are live at once:
//   RAMN_TELEMATICS_Update -> ProcessESP32Response -> SendImageCANFrame
// As locals they cost ~300 bytes of that stack. Two of them declared inline in
// RAMN_TELEMATICS_Update already overflowed the task once and left ECU D
// unresponsive -- FreeRTOS detects the overflow (configCHECK_FOR_STACK_OVERFLOW
// is 2) but vApplicationStackOverflowHook is empty, so it returns into a
// corrupted task. Keeping them off the stack is also what makes
// TELEMATICS_CAN_DEBUG safe to switch on.
//
// Safe to share: one task writes them, and RAMN_UART_SendFromTask copies into
// a stream buffer before it returns.
#ifdef ENABLE_UART
static char imgAckPrintBuf[128];
#endif
#if defined(TELEMATICS_SPI_DEBUG) || defined(TELEMATICS_CAN_DEBUG)
static char telemDbgBuf[96];
#endif

// Last 0x303 ACK payload from ECU A, verbatim, waiting to be printed by the
// periodic task. See RAMN_TELEMATICS_ProcessImageACK for the byte layout.
static volatile RAMN_Bool_t kfAckPrintNeeded = False;
static uint8_t              kfAckPayload[8]  = {0};
static uint8_t              kfAckPayloadLen  = 0U;

// Counted so the stats line answers "does ECU A ever reply" without depending
// on catching a one-shot print. kfAckMissedCnt exists because the ACK WAIT IS
// ROUTINELY CUT SHORT: the IMG_START handler sets streamState back to
// KEYFRAME_ACTIVE with no guard, and the ESP32 sends keyframes about once a
// second, well inside KF_ACK_TIMEOUT_MS (2 s). So KEYFRAME_SENT almost never
// survives long enough for the timeout branch to run, and a missing ACK
// produced no output at all -- neither an ACK line nor a timeout line.
static volatile uint32_t kfAckRxCnt     = 0U;   // 0x303 frames received, ever
static volatile uint32_t kfAckMissedCnt = 0U;   // keyframes whose ACK never came

// Dynamic poll interval: SPI_POLL_INTERVAL_MS when idle, 1 ms when streaming
static uint32_t currentPollIntervalMs = SPI_POLL_INTERVAL_MS;

// Image-mode poll request buffer: [0x03][0xBB][0x01][0xBA] + 156 dummy bytes
static uint8_t pollImageTxBuffer[SPI_RX_BUFFER_SIZE];

// RX buffer for ESP32 responses (double buffer for safety)
static uint8_t spiRxBufferA[SPI_RX_BUFFER_SIZE];
static uint8_t spiRxBufferB[SPI_RX_BUFFER_SIZE];
static uint8_t* volatile activeRxBuffer = spiRxBufferA;
static uint8_t* volatile processRxBuffer = spiRxBufferB;

// Poll request message with padding for full-duplex SPI
// Format: [LEN][START][DUMMY][CHECKSUM] followed by dummy bytes
// SPI must clock out bytes to receive bytes, so we send SPI_RX_BUFFER_SIZE bytes total
#define POLL_REQUEST_HEADER_SIZE 4
static uint8_t pollTxBuffer[SPI_RX_BUFFER_SIZE] = {
	0x02, 0xBB, 0x00, 0xBB,  // Poll request header
	// Rest initialized to 0x00 (dummy bytes for RX clocking)
};

// SPI transmission and polling statistics (following RAMN pattern)
typedef struct
{
	// TX stats (CAN → ESP32)
	volatile uint32_t spiTxRequestCnt;     // Number of CAN messages requested for SPI transmission
	volatile uint32_t spiTxSentCnt;        // Number of successful SPI transmissions (batch count)
	volatile uint32_t spiTxBytesSent;      // Total bytes successfully transmitted over SPI
	volatile uint32_t spiTxErrorCnt;       // Number of SPI transmission failures
	volatile uint32_t spiBufferOverrunCnt; // Number of messages dropped due to buffer full
	volatile uint32_t spiBufferFlushCnt;   // Number of times buffer was flushed
	volatile uint32_t spiBufferSwapCnt;    // Number of buffer swaps
	volatile HAL_StatusTypeDef lastSpiError; // Last SPI error code

	// RX stats (ESP32 → CAN)
	volatile uint32_t spiRxPollCnt;           // Number of polls sent to ESP32
	volatile uint32_t spiRxCompleteCnt;       // Successful RX completions
	volatile uint32_t spiRxEmptyRespCnt;      // Empty responses (ESP32 has no data)
	volatile uint32_t spiRxNoRespFoundCnt;    // No valid response marker found in buffer
	volatile uint32_t spiRxTimeoutCnt;        // Response timeouts
	volatile uint32_t spiRxErrorCnt;          // DMA start errors
	volatile uint32_t spiRxInvalidCnt;        // Invalid responses from ESP32
	volatile uint32_t spiRxChecksumErrorCnt;  // Checksum failures
	volatile uint32_t spiRxCANQueuedCnt;      // CAN messages queued successfully
	volatile uint32_t spiRxCANQueueFailCnt;   // CAN queue failures
	volatile uint32_t spiRxBusySkipCnt;       // Polls skipped due to SPI bus busy (TX in progress)
	volatile uint32_t spiRxStateSkipCnt;      // Polls skipped due to wrong state (stuck in REQUESTED)
	volatile uint32_t spiRxWatchdogResetCnt;  // Watchdog forced state resets
} RAMN_SPI_Stats_t;

static RAMN_SPI_Stats_t spiStats = {0};

// ============================================================================
// PRIVATE FUNCTION DECLARATIONS
// ============================================================================
static void FlushSPIBuffer(void);
static RAMN_Bool_t RequestESP32Poll(void);
static void ProcessESP32Response(void);
static void PrintSPIStats(void);
static void PrintImageACK(void);
static void PrintImageACKTimeout(void);

// ============================================================================
// INITIALIZATION
// ============================================================================
void RAMN_TELEMATICS_Init(uint32_t tick)
{
	// Initialize TX buffers
	activeBufferPos = 0;
	flushBufferSize = 0;
	spiTransmitBusy = False;
	spiTxLastFlushTick = tick;

	// Initialize RX polling
	spiPollState = SPI_POLL_IDLE;
	spiLastPollTick = tick;
	spiLastPollAttemptTick = tick;

	// Initialize stats printing
	spiStatsLastPrintTick = tick;

	// Zero-initialize RX buffers (prevent garbage on first poll)
	RAMN_memset(spiRxBufferA, 0, SPI_RX_BUFFER_SIZE);
	RAMN_memset(spiRxBufferB, 0, SPI_RX_BUFFER_SIZE);

	// Initialize image streaming state machine
	streamState           = STREAM_IDLE;
	currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
	kfAckReceived         = False;
	deltaFirstTileOfFrame = False;
	lastDeltaActivityTick = tick;

	// Image-mode poll request: [0x03][0xBB][0x01][0xBA] + zero padding
	RAMN_memset(pollImageTxBuffer, 0, SPI_RX_BUFFER_SIZE);
	pollImageTxBuffer[0] = 0x03U;
	pollImageTxBuffer[1] = 0xBBU;
	pollImageTxBuffer[2] = 0x01U;
	pollImageTxBuffer[3] = 0xB9U;
}

// ============================================================================
// DMA-BASED SPI FLUSH WITH ATOMIC BUFFER SWAP (CAN → ESP32)
// ============================================================================
// This function implements a lock-free buffer swap and DMA transmission:
// 1. Atomically swap activeBuffer <-> flushBuffer (single critical section ~1µs)
// 2. Start DMA transmission from flushBuffer (non-blocking)
// 3. CAN RX continues writing to NEW activeBuffer (zero blocking!)
//
// The SPI DMA completion callback (HAL_SPI_TxCpltCallback) will:
// - Clear spiTransmitBusy flag
// - Update transmission statistics
// ============================================================================
static void FlushSPIBuffer(void)
{
	extern SPI_HandleTypeDef hspi2;
	HAL_StatusTypeDef status;
	uint16_t bytesToSend = 0;
	uint8_t* bufferToFlush = NULL;

	// ===== NEW OWNERSHIP CHECK =====
	if (spiOwner != SPI_OWNER_NONE)
		return;  // SPI in use by TX or POLL

	// Check if DMA transmission is already in progress
	if (spiTransmitBusy == True)
		return;  // Previous transmission still busy - skip this flush

	// CRITICAL SECTION: Atomic buffer swap (~1µs duration)
	// This is the ONLY critical section in the entire CAN-to-SPI pipeline!
	taskENTER_CRITICAL();

	// Check if active buffer has data to send
	if (activeBufferPos == 0)
	{
		taskEXIT_CRITICAL();
		return;  // Nothing to send
	}

	// Atomic pointer swap: activeBuffer <-> flushBuffer
	uint8_t* temp = activeBuffer;
	activeBuffer = flushBuffer;
	flushBuffer = temp;

	// Transfer size information
	flushBufferSize = activeBufferPos;
	bytesToSend = flushBufferSize;
	bufferToFlush = flushBuffer;

	// Reset active buffer for new writes (CAN RX can now write immediately!)
	activeBufferPos = 0;

	// Mark SPI as busy BEFORE starting DMA
	spiOwner = SPI_OWNER_TX;
	spiTransmitBusy = True;

	// Update statistics
	spiStats.spiBufferFlushCnt++;
	spiStats.spiBufferSwapCnt++;

	taskEXIT_CRITICAL();
	// END CRITICAL SECTION - Total duration: ~1µs for pointer swap

	// Start DMA transmission (non-blocking, interrupts enabled)
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit_DMA(&hspi2, bufferToFlush, bytesToSend);

	if (status != HAL_OK)
	{
		// DMA start failed - immediately release resources
		HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

		taskENTER_CRITICAL();
         spiTransmitBusy = False;
         spiOwner = SPI_OWNER_NONE;
         spiStats.spiTxErrorCnt++;
         spiStats.lastSpiError = status;
         taskEXIT_CRITICAL();
	}
	// If DMA started successfully, HAL_SPI_TxCpltCallback will handle cleanup
}

// ============================================================================
// SPI DMA TRANSMISSION COMPLETE CALLBACK - CUSTOM EXPANSION BOARD
// ============================================================================
// This callback is triggered when our custom CAN-to-SPI DMA transmission completes.
// It is called FROM the main HAL_SPI_TxCpltCallback in ramn_spi.c
// WARNING: Called from ISR context - keep it fast!
// ============================================================================
void RAMN_TELEMATICS_SPI_TxCpltCallback(void)
{
	// De-assert chip select for custom expansion board
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

	// Update statistics (safe in ISR - simple increments)
	spiStats.spiTxSentCnt++;
	spiStats.spiTxBytesSent += flushBufferSize;

	// Clear busy flag - allow next flush
	spiTransmitBusy = False;
	spiOwner = SPI_OWNER_NONE;
}

// ============================================================================
// SPI DMA TRANSMIT-RECEIVE COMPLETE CALLBACK FOR ESP32 POLLING
// ============================================================================
// This callback is triggered when HAL_SPI_TransmitReceive_DMA completes.
// It is called FROM the HAL_SPI_TxRxCpltCallback in ramn_spi.c
// WARNING: Called from ISR context - keep it fast!
//
// RACE CONDITION PROTECTION:
// This callback can fire concurrently with the timeout handler in Update().
// We use atomic state check and update to prevent corruption.
// ============================================================================
void RAMN_TELEMATICS_SPI_TxRxCpltCallback(void)
{
	// This is called when simultaneous TX/RX DMA completes (polling operation)

	// CRITICAL: Always de-assert chip select first (safety measure)
	// This ensures CS is released even if state is unexpected
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

	// Check state and update atomically to prevent race with timeout handler
	if (spiPollState == SPI_POLL_REQUESTED)
	{
		// Successfully received response from ESP32
		spiPollState = SPI_POLL_COMPLETE;
		spiOwner = SPI_OWNER_NONE;
		spiPollCompleteTick = xTaskGetTickCount();  // Record when we entered COMPLETE state

		// Update statistics
		spiStats.spiRxCompleteCnt++;
	}
	// If state != REQUESTED, this is likely a late callback after timeout
	// CS is already de-asserted above, so no harm done
}

// ============================================================================
// REQUEST POLL FROM ESP32 (NON-BLOCKING)
// ============================================================================
// This function initiates a TransmitReceive DMA operation:
// - Sends poll request to ESP32
// - Simultaneously receives response into RX buffer
// - Returns immediately (DMA handles the transfer)
// ============================================================================
static RAMN_Bool_t RequestESP32Poll(void)
{
	extern SPI_HandleTypeDef hspi2;
	HAL_StatusTypeDef status;

	// Check if we're already waiting for a response
	if (spiPollState != SPI_POLL_IDLE)
	{
		spiStats.spiRxStateSkipCnt++;
		return False;  // Already polling
	}

	// Check if SPI bus is busy (CAN→SPI transmission in progress)
	if (spiOwner != SPI_OWNER_NONE)
	{
		spiStats.spiRxBusySkipCnt++;
		return False;  // Try again later
	}

	// Swap RX buffers (similar to TX double-buffer pattern)
	taskENTER_CRITICAL();
	uint8_t* temp = activeRxBuffer;
	activeRxBuffer = processRxBuffer;
	processRxBuffer = temp;
	spiPollState = SPI_POLL_REQUESTED;
	spiOwner = SPI_OWNER_POLL;
	spiPollRequestTick = xTaskGetTickCount();
	spiStats.spiRxPollCnt++;
	taskEXIT_CRITICAL();

	// Select poll request type: image-mode (0x01) when streaming, normal (0x00) otherwise
	//uint8_t* txBuf = (streamState != STREAM_IDLE) ? pollImageTxBuffer : pollTxBuffer;

	static uint8_t peek_counter = 0;
	uint8_t* txBuf = pollTxBuffer;

	if (streamState != STREAM_IDLE) {
    	txBuf = pollImageTxBuffer;
	} else if (++peek_counter >= 10) { // Every 10th poll, check for images
    	txBuf = pollImageTxBuffer;
    	peek_counter = 0;
	}

	// Start simultaneous TX/RX DMA operation
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive_DMA(&hspi2,
	                                      txBuf,
	                                      activeRxBuffer,
	                                      SPI_RX_BUFFER_SIZE);

	if (status != HAL_OK)
	{
		// DMA start failed - reset state
		HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

		taskENTER_CRITICAL();
		spiPollState = SPI_POLL_IDLE;
		spiOwner = SPI_OWNER_NONE;
		spiStats.spiRxErrorCnt++;
		taskEXIT_CRITICAL();

		return False;
	}

	return True;  // Poll request sent successfully
}

// ============================================================================
// PROCESS ESP32 POLL RESPONSE
// ============================================================================
// Parse the response from ESP32 and queue CAN messages
// Expected format: [LEN][0xCC][CAN_ID_0-3][DLC][FLAGS][DATA...][CHECKSUM]
// Empty response (no data): [0x02][0xCC][0x00][0xCC]
//
// CONSTRAINT: Maximum ONE CAN message per poll
// Max size: 73 bytes (1+1+4+1+1+64+1)
//
// SPI FULL-DUPLEX NOTE:
// Due to SPI full-duplex operation, ESP32's response doesn't start at byte 0.
// We scan for the 0xCC marker to find where the response begins.
// ============================================================================
// ============================================================================
// HELPER: Build and send a CAN-FD frame for image streaming
// ============================================================================
static void SendImageCANFrame(uint32_t canId, uint32_t dlc,
                               RAMN_Bool_t brs, const uint8_t* data)
{
	FDCAN_TxHeaderTypeDef h;
	h.Identifier          = canId;
	h.IdType              = FDCAN_STANDARD_ID;
	h.TxFrameType         = FDCAN_DATA_FRAME;
	h.DataLength          = dlc;
	h.BitRateSwitch       = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
	h.FDFormat            = FDCAN_FD_CAN;
	h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	h.MessageMarker       = 0U;
	h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

	RAMN_Result_t result = RAMN_FDCAN_SendMessage(&h, data);
	if (result == RAMN_OK)
		spiStats.spiRxCANQueuedCnt++;
	else
		spiStats.spiRxCANQueueFailCnt++;

#ifdef TELEMATICS_CAN_DEBUG
	{
		uint8_t payLen = DLCtoUINT8(dlc);
		// Print first 8 bytes of payload in one shot — same style as SPI debug
		int  canLen = snprintf(telemDbgBuf, sizeof(telemDbgBuf),
		    "CAN TX: ID=0x%03lX BRS=%d len=%u %s | %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
		    (unsigned long)canId, (int)(brs == True), payLen,
		    (result == RAMN_OK) ? "OK" : "FAIL",
		    (payLen > 0U) ? data[0] : 0U, (payLen > 1U) ? data[1] : 0U,
		    (payLen > 2U) ? data[2] : 0U, (payLen > 3U) ? data[3] : 0U,
		    (payLen > 4U) ? data[4] : 0U, (payLen > 5U) ? data[5] : 0U,
		    (payLen > 6U) ? data[6] : 0U, (payLen > 7U) ? data[7] : 0U);
		if (canLen > 0) RAMN_UART_SendFromTask((uint8_t*)telemDbgBuf, (uint32_t)canLen);
	}
#endif
}

// ============================================================================
// PROCESS ESP32 POLL RESPONSE — up to 2 messages per 160-byte poll
// ============================================================================
// Protocol: [LEN][0xCC][TYPE][...payload...][CHK]
//   TYPE 0x00 : CAN forward (ID at bytes 3-6, DLC at 7, FLAGS at 8, data at 9+)
//   TYPE 0x01 : IMG_START   keyframe begin
//   TYPE 0x02 : IMG_CHUNK   keyframe pixel data (RLE)
//   TYPE 0x03 : IMG_END     keyframe complete
//   TYPE 0x04 : IMG_ABORT   abort current keyframe
//   TYPE 0x10 : DELTA_FRAME one RLE tile chunk
//   TYPE 0x11 : DELTA_FRAME_END signals end of one delta frame
// Empty response: LEN=2, [0xCC][0x00][CHK]
// ============================================================================
/* Byte count -> FDCAN DLC enum.
 *
 * The DLC byte on the SPI link is a BYTE COUNT (0-64). This HAL's DataLength
 * is an ENUM (FDCAN_DLC_BYTES_64 == 0x0F), and RAMN_FDCAN_SendMessage calls
 * DLCtoUINT8 on it to recover the count. Assigning the wire's byte count
 * straight into DataLength therefore made the send path index a sixteen-entry
 * table with a number up to 64 -- an out-of-bounds read 48 bytes past
 * DlcToUint8convTable for a full CAN FD frame.
 *
 * It went unnoticed because below 9 the enum and the byte count are the same
 * number, and almost all real traffic is DLC <= 8.
 *
 * Sizes CAN FD cannot express (9, 10, 11, 13, 14, 15, 17-19, ...) return 0xFF
 * so the caller can refuse them. Rounding up would put bytes on the bus that
 * the sender never wrote; rounding down would truncate.
 */
static uint8_t ByteCountToDLC(uint8_t bytes)
{
	for (uint8_t enumVal = 0U; enumVal < 16U; enumVal++)
	{
		if (DLCtoUINT8(enumVal) == bytes) return enumVal;
	}
	return 0xFFU;
}

static void ProcessESP32Response(void)
{
	uint8_t* rxBuf  = processRxBuffer;
	uint16_t offset = 0;

#ifdef TELEMATICS_SPI_DEBUG
	// Print first 16 raw bytes of every poll response
	int  dbgLen = snprintf(telemDbgBuf, sizeof(telemDbgBuf),
	    "SPI RX: %02X %02X %02X %02X %02X %02X %02X %02X "
	             "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
	    rxBuf[0],  rxBuf[1],  rxBuf[2],  rxBuf[3],
	    rxBuf[4],  rxBuf[5],  rxBuf[6],  rxBuf[7],
	    rxBuf[8],  rxBuf[9],  rxBuf[10], rxBuf[11],
	    rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15]);
	if (dbgLen > 0) RAMN_UART_SendFromTask((uint8_t*)telemDbgBuf, (uint32_t)dbgLen);
#endif

	for (int msgNum = 0; msgNum < 2; msgNum++)
	{
		// Scan from current offset for a 0xCC marker preceded by a valid LEN byte
		RAMN_Bool_t found     = False;
		uint16_t    respStart = 0;

		for (uint16_t i = offset; i < SPI_RX_BUFFER_SIZE - 1U; i++)
		{
			if (rxBuf[i] == 0xCCU && i > 0U)
			{
				uint8_t candidateLen = rxBuf[i - 1U];
				if (candidateLen >= 2U && candidateLen <= 80U &&
				    (i - 1U + 1U + candidateLen) <= SPI_RX_BUFFER_SIZE)
				{
					respStart = i - 1U;
					found     = True;
					break;
				}
			}
		}

		if (!found)
		{
			if (msgNum == 0)
			{
				spiStats.spiRxNoRespFoundCnt++;
#ifdef TELEMATICS_SPI_DEBUG
				RAMN_UART_SendStringFromTask("SPI: no 0xCC marker found\r\n");
#endif
			}
			break;
		}

		uint8_t* msg    = &rxBuf[respStart];
		uint8_t  msgLen = msg[0];

		// Advance offset past this message for the next iteration
		offset = respStart + 1U + msgLen;

		// Empty response: type 0x00 with LEN=2
		if (msgLen == 2U && msg[1] == 0xCCU && msg[2] == 0x00U)
		{
			spiStats.spiRxEmptyRespCnt++;
			continue;
		}

		// Require at least [0xCC][TYPE][CHK] = 3 bytes in the payload
		if (msgLen < 3U || msg[1] != 0xCCU)
		{
			spiStats.spiRxInvalidCnt++;
			continue;
		}

		// Verify XOR checksum over bytes msg[1]..msg[msgLen]
		uint8_t chk = 0U;
		for (uint8_t k = 1U; k <= msgLen; k++) chk ^= msg[k];
		if (chk != 0U)
		{
			spiStats.spiRxChecksumErrorCnt++;
#ifdef TELEMATICS_SPI_DEBUG
			int  chkLen = snprintf(telemDbgBuf, sizeof(telemDbgBuf),
			    "SPI: chk fail at %u len=%u residue=0x%02X\r\n",
			    respStart, msgLen, chk);
			if (chkLen > 0) RAMN_UART_SendFromTask((uint8_t*)telemDbgBuf, (uint32_t)chkLen);
#endif
			continue;
		}

		uint8_t msgType = msg[2];

#ifdef TELEMATICS_SPI_DEBUG
		{
			int  typeLen = snprintf(telemDbgBuf, sizeof(telemDbgBuf),
			    "SPI: msg%d at %u len=%u type=0x%02X\r\n",
			    msgNum, respStart, msgLen, msgType);
			if (typeLen > 0) RAMN_UART_SendFromTask((uint8_t*)telemDbgBuf, (uint32_t)typeLen);
		}
#endif

		// ------------------------------------------------------------------
		// CAN forward — relay as classic CAN to the bus
		// Format: [LEN][0xCC][ID3][ID2][ID1][ID0][DLC][FLAGS][DATA...][CHK]
		//
		// A CAN poll response carries NO type byte. Byte 2 is ID[31:24], and
		// that is exactly why every type code sits at or above 0x20: an
		// extended identifier caps at 0x1FFFFFFF, so byte 2 of a CAN frame
		// never exceeds RAMN_MAX_ID_HIGH_BYTE. Testing `<=` against that
		// ceiling is the dispatch rule, and it is what makes byte 2
		// self-describing.
		//
		// This branch used to require byte 2 == 0x00 and read every field one
		// position later, as though a type byte were present. Two failures
		// followed, both seen on hardware:
		//
		//   every field shifted by one -- ID 0x100 with DLC 8 was read as
		//   identifier 0x00010008, which the HAL truncates to 0x008, and the
		//   misread FLAGS byte set the remote bit, so the frame went out
		//   empty. Every cansend from the UI appeared as ID 0x008, no payload.
		//
		//   extended identifiers above 0x00FFFFFF have a nonzero byte 2, so
		//   they matched no branch at all and were dropped silently. J1939
		//   traffic (0x18FEE000 and friends) never reached the bus.
		// ------------------------------------------------------------------
		if (msgType <= RAMN_MAX_ID_HIGH_BYTE)
		{
			/* MSGLEN counts marker + ID(4) + DLC + FLAGS + CHK = 8, plus the
			 * payload. A zero-payload frame -- any remote frame, or DLC 0 --
			 * is exactly 8. */
			if (msgLen < 8U) { spiStats.spiRxInvalidCnt++; continue; }

			uint32_t canId = ((uint32_t)msg[2] << 24) | ((uint32_t)msg[3] << 16) |
			                 ((uint32_t)msg[4] << 8)  |  (uint32_t)msg[5];
			uint8_t dlc   = msg[6];
			uint8_t flags = msg[7];

			/* The wire carries a byte count; DataLength is an enum. */
			uint8_t dlcEnum = ByteCountToDLC(dlc);
			if (dlcEnum == 0xFFU) { spiStats.spiRxInvalidCnt++; continue; }

			FDCAN_TxHeaderTypeDef h;
			h.Identifier          = canId;
			h.IdType              = (flags & 0x01U) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
			h.TxFrameType         = (flags & 0x02U) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
			h.DataLength          = dlcEnum;
			h.BitRateSwitch       = FDCAN_BRS_OFF;
			h.FDFormat            = FDCAN_CLASSIC_CAN;
			h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
			h.MessageMarker       = 0U;
			h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

			uint8_t data[64];
			RAMN_memset(data, 0, sizeof(data));
			/* The DLC byte on this link is a BYTE COUNT (0-64), not an
			 * FDCAN DLC enum. DLCtoUINT8 indexes a sixteen-entry table, so
			 * feeding it a byte count reads out of bounds for anything above
			 * 8 -- a 64-byte CAN FD payload indexed 48 bytes past the end of
			 * DlcToUint8convTable. It went unnoticed because for 0-8 the enum
			 * and the byte count are the same number, and real traffic is
			 * almost all DLC <= 8. */
			uint8_t payLen = (dlc > CAN_MAX_PAYLOAD_BYTES) ? CAN_MAX_PAYLOAD_BYTES : dlc;
			uint8_t maxPay = (msgLen >= 8U) ? (msgLen - 8U) : 0U;
			if (payLen > maxPay) payLen = maxPay;
			for (uint8_t k = 0U; k < payLen; k++) data[k] = msg[8U + k];

			RAMN_Result_t fwdResult = RAMN_FDCAN_SendMessage(&h, data);
			if (fwdResult == RAMN_OK) spiStats.spiRxCANQueuedCnt++;
			else spiStats.spiRxCANQueueFailCnt++;

#ifdef TELEMATICS_CAN_DEBUG
			{
				int  canLen = snprintf(telemDbgBuf, sizeof(telemDbgBuf),
				    "CAN TX: ID=0x%03lX BRS=0 len=%u %s | %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
				    (unsigned long)canId, payLen,
				    (fwdResult == RAMN_OK) ? "OK" : "FAIL",
				    (payLen > 0U) ? data[0] : 0U, (payLen > 1U) ? data[1] : 0U,
				    (payLen > 2U) ? data[2] : 0U, (payLen > 3U) ? data[3] : 0U,
				    (payLen > 4U) ? data[4] : 0U, (payLen > 5U) ? data[5] : 0U,
				    (payLen > 6U) ? data[6] : 0U, (payLen > 7U) ? data[7] : 0U);
				if (canLen > 0) RAMN_UART_SendFromTask((uint8_t*)telemDbgBuf, (uint32_t)canLen);
			}
#endif
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x81: IMG_START — start of a keyframe
		// Format: [0x0A][0xCC][0x81][W_HI][W_LO][H_HI][H_LO][CH_HI][CH_LO][X_OFF][Y_OFF][CHK]
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_IMG_START)
		{
			if (msgLen < 10U) { spiStats.spiRxInvalidCnt++; continue; }

			uint16_t w  = (uint16_t)((uint16_t)msg[4] | ((uint16_t)msg[3] << 8));
			uint16_t h  = (uint16_t)((uint16_t)msg[6] | ((uint16_t)msg[5] << 8));
			uint16_t ch = (uint16_t)((uint16_t)msg[8] | ((uint16_t)msg[7] << 8));
			uint8_t  xo = msg[9];
			uint8_t  yo = msg[10];

			// A new keyframe while the previous one is still waiting for its
			// ACK means that ACK never arrived -- and the timeout branch in
			// RAMN_TELEMATICS_Update will never run to say so, because this
			// line is what stops it. Count it here instead.
			if ((streamState == KEYFRAME_SENT) && (kfAckReceived == False)) kfAckMissedCnt++;

			kfTotalChunks = ch;
			kfChunksSent  = 0U;
			kfXOffset     = xo;
			kfYOffset     = yo;
			streamState   = KEYFRAME_ACTIVE;
			currentPollIntervalMs = 1U;

			// Forward 0x300 IMG_START to ECU A
			uint8_t canData[12];
			canData[0]  = (uint8_t)(w & 0xFFU);
			canData[1]  = (uint8_t)(w >> 8);
			canData[2]  = (uint8_t)(h & 0xFFU);
			canData[3]  = (uint8_t)(h >> 8);
			canData[4]  = (uint8_t)(ch & 0xFFU);
			canData[5]  = (uint8_t)(ch >> 8);
			canData[6]  = xo;
			canData[7]  = yo;
			canData[8]  = 0x00U;
			canData[9]  = 0x00U;
			canData[10] = 0x01U;   // VERSION
			uint8_t xorChk = 0U;
			for (uint8_t k = 0U; k < 11U; k++) xorChk ^= canData[k];
			canData[11] = xorChk;
			SendImageCANFrame(IMG_CAN_ID_START, FDCAN_DLC_BYTES_12, False, canData);
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x82: IMG_CHUNK — keyframe RLE pixel data
		// Format: [LEN][0xCC][0x82][SEQ_HI][SEQ_LO][PAYLOAD_LEN][RLE...][CHK]
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_IMG_CHUNK)
		{
			if (streamState != KEYFRAME_ACTIVE) continue;
			if (msgLen < 5U) { spiStats.spiRxInvalidCnt++; continue; }

			uint8_t payLen = msg[5];
			if (payLen == 0U || 6U + payLen > msgLen) { spiStats.spiRxInvalidCnt++; continue; }

			// Forward as one or two 0x301 IMG_DATA frames (CAN-FD + BRS, DLC
			// always 64). Each frame carries at most 61 real bytes — 64 minus
			// a 3-byte header [SEQ_HI][SEQ_LO][REAL_LEN]. A full 64-byte SPI
			// chunk needs TWO frames to cross without loss: sending it in one
			// fixed-64 frame with a 2-byte header used to silently truncate
			// the last 2 bytes of every full chunk (bug found 2026-09-01).
			// REAL_LEN also lets ECU A recover the true byte count of the
			// final, short chunk of a keyframe instead of assuming a fixed
			// 62 and feeding its own zero-padding into the decoder as if it
			// were real pixel data.
			uint8_t srcOffset = 0U;
			while (srcOffset < payLen)
			{
				uint8_t remaining = (uint8_t)(payLen - srcOffset);
				uint8_t frameLen  = (remaining <= 61U) ? remaining : 61U;

				uint8_t canData[64];
				RAMN_memset(canData, 0, sizeof(canData));
				canData[0] = msg[3];    // SEQ_HI
				canData[1] = msg[4];    // SEQ_LO
				canData[2] = frameLen;  // REAL_LEN -- true byte count carried in this frame
				for (uint8_t k = 0U; k < frameLen; k++) canData[3U + k] = msg[6U + srcOffset + k];
				SendImageCANFrame(IMG_CAN_ID_DATA, FDCAN_DLC_BYTES_64, True, canData);

				srcOffset = (uint8_t)(srcOffset + frameLen);
			}
			kfChunksSent++;
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x83: IMG_END — keyframe complete
		// Format: [0x83][0xCC][0x83][CHK]
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_IMG_END)
		{
			if (streamState != KEYFRAME_ACTIVE) continue;

			// Forward 0x302 IMG_END
			uint8_t canData[8];
			RAMN_memset(canData, 0, sizeof(canData));
			canData[0] = (uint8_t)(kfChunksSent & 0xFFU);
			canData[1] = (uint8_t)(kfChunksSent >> 8);
			// CRC16 bytes (2-3) left as 0x00 — full CRC computation is optional
			canData[4] = 0x00U;   // STATUS OK
			SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_8, False, canData);

			streamState   = KEYFRAME_SENT;
			kfAckReceived = False;
			kfAckWaitTick = xTaskGetTickCount();
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x84: IMG_ABORT — abort current keyframe
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_IMG_ABORT)
		{
			if (streamState == KEYFRAME_ACTIVE || streamState == KEYFRAME_SENT)
			{
				uint8_t canData[8];
				RAMN_memset(canData, 0, sizeof(canData));
				canData[4] = 0x01U;   // STATUS abort
				SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_8, False, canData);
			}
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x90: DELTA_FRAME — one RLE chunk of a dirty tile
		// Format: [LEN][0xCC][0x10][TILE_X][TILE_Y][TILE_SIZE][CHUNK_SEQ][PAYLOAD_LEN][RLE...][CHK]
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_DELTA_FRAME)
		{
			if (msgLen < 7U) { spiStats.spiRxInvalidCnt++; continue; }

			uint8_t tileX    = msg[3];
			uint8_t tileY    = msg[4];
			uint8_t tileSize = msg[5];
			uint8_t chunkSeq = msg[6];
			uint8_t payLen   = msg[7];

			if (payLen == 0U || 8U + payLen > msgLen) { spiStats.spiRxInvalidCnt++; continue; }

			// Send DELTA_FRAME_START (0x304) before the first tile chunk of a frame
			if (deltaFirstTileOfFrame == False && (chunkSeq & 0x7FU) == 0U)
			{
				deltaFirstTileOfFrame = True;
				deltaFrameSeq++;
				deltaTileCount = 0U;

				uint8_t startData[8];
				RAMN_memset(startData, 0, sizeof(startData));
				startData[0] = deltaFrameSeq;
				startData[1] = 0U;   // tile_count filled in DELTA_FRAME_END
				SendImageCANFrame(DELTA_CAN_ID_FRAME_START, FDCAN_DLC_BYTES_8, False, startData);

				if (streamState != DELTA_ACTIVE)
				{
					streamState           = DELTA_ACTIVE;
					currentPollIntervalMs = 1U;
				}
			}

			// Track tiles (each tile signals last chunk)
			if (chunkSeq & 0x80U) deltaTileCount++;
			lastDeltaActivityTick = xTaskGetTickCount();

			// Forward as 0x305 DELTA_TILE_CHUNK (CAN-FD + BRS, DLC=64)
			uint8_t canData[64];
			RAMN_memset(canData, 0, sizeof(canData));
			canData[0] = tileX;
			canData[1] = tileY;
			canData[2] = tileSize;
			canData[3] = chunkSeq;
			uint8_t copyLen = (payLen <= 59U) ? payLen : 59U;
			canData[4] = copyLen;
			for (uint8_t k = 0U; k < copyLen; k++) canData[5U + k] = msg[8U + k];
			SendImageCANFrame(DELTA_CAN_ID_TILE_CHUNK, FDCAN_DLC_BYTES_64, True, canData);
			continue;
		}

		// ------------------------------------------------------------------
		// TYPE 0x91: DELTA_FRAME_END — end of one delta frame
		// Format: [0x04][0xCC][0x11][TILE_COUNT][CHK]
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_DELTA_FRAME_END)
		{
			// Forward 0x306 DELTA_FRAME_END
			uint8_t canData[4];
			canData[0] = deltaFrameSeq;
			canData[1] = 0x00U;   // STATUS complete
			canData[2] = 0x00U;
			canData[3] = 0x00U;
			SendImageCANFrame(DELTA_CAN_ID_FRAME_END, FDCAN_DLC_BYTES_4, False, canData);

			deltaFirstTileOfFrame = False;   // ready for next delta frame's START
			continue;
		}

		// Unknown type
		spiStats.spiRxInvalidCnt++;
	}
}

// ============================================================================
// CAN MESSAGE RECEPTION WITH DOUBLE-BUFFERING AND MINIMAL LOCKING
// ============================================================================
// Called when a CAN message is received (Hardware filters should be configured separately in ramn_canfd.c; with recvStdCANIDList and recvExtCANIDList)
// Note that by default, ECU A has no filter.
// This function is called from a task using an intermediary CAN buffer, so it does not need to return quickly.
//
// CRITICAL PERFORMANCE OPTIMIZATION:
// - Message encoding: Done in local buffer (no locks, ~5µs)
// - Buffer write: Short critical section (~5µs for memcpy)
// - Flush trigger: Non-blocking call to FlushSPIBuffer (no waiting)
//
// CRITICAL SECTION JUSTIFICATION:
// We MUST protect the read of activeBuffer + activeBufferPos + the write operation
// as a single atomic unit. If buffer swap happens between reading the pointer and
// writing, we could write to the wrong buffer or corrupt position tracking.
// This is unavoidable but still 20,000x better than the old 100ms blocking approach.
// ============================================================================
void RAMN_TELEMATICS_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick)
{
	// Route IMG_ACK (0x303) from ECU A to the image stream handler
	if (pHeader->Identifier == IMG_CAN_ID_ACK &&
	    pHeader->IdType == FDCAN_STANDARD_ID &&
	    pHeader->RxFrameType == FDCAN_DATA_FRAME)
	{
		RAMN_TELEMATICS_ProcessImageACK(pHeader, data, tick);
		return;
	}

	uint8_t msgBuf[73];  // MsgLen + Start + ID(4) + Len + Flags + Data(64 max) + Checksum
	uint8_t offset = 0;
	uint8_t checksum = 0;

	// Get payload size
	uint8_t payloadSize = DLCtoUINT8(pHeader->DataLength);

	// Binary format: [MSGLEN][START][ID0][ID1][ID2][ID3][LEN][FLAGS][DATA...][CHECKSUM]
	// MSGLEN: Total message length (excluding MSGLEN byte itself) for frame parsing
	// START: 0xAA (frame start marker for sync)
	// ID0-ID3: 32-bit CAN ID (big-endian, supports standard 11-bit and extended 29-bit)
	// LEN: CAN payload length (0-64)
	// FLAGS: Bit 0: ID type (0=standard, 1=extended), Bit 1: Frame type (0=data, 1=remote)
	// DATA: 0-64 bytes of CAN payload
	// CHECKSUM: Simple XOR checksum of all bytes from START to end of DATA

	// Reserve space for message length (will fill in later)
	offset++;

	msgBuf[offset++] = 0xAA;  // Start byte

	// CAN ID (32-bit, big-endian)
	msgBuf[offset++] = (pHeader->Identifier >> 24) & 0xFF;
	msgBuf[offset++] = (pHeader->Identifier >> 16) & 0xFF;
	msgBuf[offset++] = (pHeader->Identifier >> 8) & 0xFF;
	msgBuf[offset++] = pHeader->Identifier & 0xFF;

	// Payload length
	msgBuf[offset++] = payloadSize;

	// Flags byte
	uint8_t flags = 0;
	if (pHeader->IdType == FDCAN_EXTENDED_ID) flags |= 0x01;
	if (pHeader->RxFrameType == FDCAN_REMOTE_FRAME) flags |= 0x02;
	msgBuf[offset++] = flags;

	// Copy CAN payload data
	for (uint8_t i = 0; i < payloadSize && i < 64; i++)
	{
		msgBuf[offset++] = data[i];
	}

	// Calculate checksum (XOR of all bytes from START onwards)
	for (uint8_t i = 1; i < offset; i++)
	{
		checksum ^= msgBuf[i];
	}
	msgBuf[offset++] = checksum;

	// Fill in message length (total bytes excluding MSGLEN itself)
	msgBuf[0] = offset - 1;

	// ========================================================================
	// BUFFER WRITE WITH MINIMAL CRITICAL SECTION
	// ========================================================================
	// CRITICAL FIX: Must atomically read position AND buffer pointer together
	// Otherwise buffer swap can happen between reading pointer and position,
	// causing us to write to the wrong buffer or corrupt the position counter.
	//
	// This critical section is MUCH shorter than the old blocking SPI approach:
	// - Old: 100ms (blocking SPI transmission)
	// - New: ~5µs (just the memcpy operation)
	// ========================================================================

	taskENTER_CRITICAL();

	// Atomically capture current buffer state
	uint16_t currentPos = activeBufferPos;
	uint8_t* targetBuffer = activeBuffer;

	// Check if message will fit in current buffer
	if (currentPos + offset <= SPI_TX_BUFFER_SIZE)
	{
		// Fast memcpy - safe in critical section (~5µs for 73 bytes max)
		for (uint8_t i = 0; i < offset; i++)
		{
			targetBuffer[currentPos + i] = msgBuf[i];
		}

		// Update buffer position
		activeBufferPos = currentPos + offset;

		// Update statistics
		spiStats.spiTxRequestCnt++;
	}
	else
	{
		// Buffer overflow - message dropped
		// This is extremely rare with double-buffering (only if both buffers full)
		spiStats.spiBufferOverrunCnt++;
		spiStats.spiTxRequestCnt++;  // Still count the request
	}

	taskEXIT_CRITICAL();
	// END CRITICAL SECTION - Duration: ~5µs for message copy

	// Check if we should trigger a flush (non-blocking check, no lock needed)
	uint16_t currentBufferPos = activeBufferPos;  // Volatile read
	if (currentBufferPos >= SPI_BUFFER_FLUSH_THRESHOLD ||
	    (tick - spiTxLastFlushTick) >= SPI_FLUSH_INTERVAL_MS)
	{
		// Trigger flush (non-blocking - just starts DMA if not busy)
		FlushSPIBuffer();
		spiTxLastFlushTick = tick;
	}
}

// ============================================================================
// PRINT SPI STATISTICS TO UART
// ============================================================================
// Prints detailed SPI transmission and polling statistics over UART
// Called periodically to monitor CAN-to-SPI bridge performance
// ============================================================================
// ============================================================================
// IMAGE ACK REPORTING
//
// These print from RAMN_TELEMATICS_Update, which runs on the periodic task.
// That task's whole stack is 1 KB -- RAMN_PeriodicBuffer[256] in main.c -- and
// PrintSPIStats already puts a ~600-byte frame on it. Two char buffers declared
// inline in RAMN_TELEMATICS_Update grew its frame from 32 to 224 bytes, and
// with PrintSPIStats' frame live underneath it that overflowed the task.
// configCHECK_FOR_STACK_OVERFLOW is 2, so FreeRTOS detected it -- and
// vApplicationStackOverflowHook is empty, so it returned into a corrupted
// task and ECU D went dead on hardware.
//
// So: one static buffer, not a local, and a separate frame that is never live
// at the same time as PrintSPIStats'. Both printers run only on the periodic
// task and RAMN_UART_SendFromTask copies into a stream buffer before it
// returns, so sharing the buffer between them is safe.
// ============================================================================

static void PrintImageACK(void)
{
#ifdef ENABLE_UART
	if (kfAckPrintNeeded == False) return;
	kfAckPrintNeeded = False;

	int len;
	if (kfAckPayloadLen >= 8U)
	{
		uint32_t decoded = (uint32_t)kfAckPayload[2]
		                 | ((uint32_t)kfAckPayload[3] << 8)
		                 | ((uint32_t)kfAckPayload[4] << 16);
		len = snprintf(imgAckPrintBuf, sizeof(imgAckPrintBuf),
		    "ECUA ACK: st=%u flags=0x%02X decoded=%lu/115200 rx=%u drop=%u/%u (sent %u)\r\n",
		    kfAckPayload[0], kfAckPayload[1], (unsigned long)decoded,
		    kfAckPayload[5], kfAckPayload[6], kfAckPayload[7], kfChunksSent);
	}
	else
	{
		// A short ACK means ECU A predates the diagnostic payload -- say so
		// rather than printing nothing.
		len = snprintf(imgAckPrintBuf, sizeof(imgAckPrintBuf),
		    "ECUA ACK: len=%u st=%u (no diagnostics -- old ECU A build?)\r\n",
		    kfAckPayloadLen, (kfAckPayloadLen > 0U) ? kfAckPayload[0] : 0xFFU);
	}
	if (len > 0) RAMN_UART_SendFromTask((uint8_t*)imgAckPrintBuf, (uint32_t)len);
#endif
}

static void PrintImageACKTimeout(void)
{
#ifdef ENABLE_UART
	int len = snprintf(imgAckPrintBuf, sizeof(imgAckPrintBuf),
	    "ECUA ACK: TIMEOUT after %ums (sent %u chunks)\r\n",
	    (unsigned)KF_ACK_TIMEOUT_MS, kfChunksSent);
	if (len > 0) RAMN_UART_SendFromTask((uint8_t*)imgAckPrintBuf, (uint32_t)len);
#endif
}

static void PrintSPIStats(void)
{
#ifdef ENABLE_UART
	char buffer[256];  // Reduced buffer size
	int len;

	// Capture stats atomically to prevent corruption during printing
	taskENTER_CRITICAL();
	RAMN_SPI_Stats_t statsSnapshot = spiStats;
	SPI_PollState_t currentState = spiPollState;
	taskEXIT_CRITICAL();

	// Get CAN TX queue usage
	size_t canTxQueueUsed = xStreamBufferBytesAvailable(CANTxDataStreamBufferHandle);
	size_t canTxQueueFree = xStreamBufferSpacesAvailable(CANTxDataStreamBufferHandle);
	size_t canTxQueueTotal = canTxQueueUsed + canTxQueueFree;
	uint8_t canTxQueuePercent = (canTxQueueTotal > 0) ? ((canTxQueueUsed * 100) / canTxQueueTotal) : 0;

	// State names for debugging
	const char* stateNames[] = {"IDLE", "REQ", "DONE", "TO"};
	const char* stateName = (currentState <= SPI_POLL_TIMEOUT) ? stateNames[currentState] : "UNK";

	// Print compact stats on single line to reduce UART load
	len = snprintf(buffer, sizeof(buffer),
		"SPI: TX[Req:%lu Sent:%lu Err:%lu] RX[Poll:%lu OK:%lu Empty:%lu NoResp:%lu Skip:%lu WD:%lu St:%s Q:%lu QFail:%lu] CANTxQ:%u%%  StreamState:%u ECUAack:%lu miss:%lu\r\n",
		statsSnapshot.spiTxRequestCnt,
		statsSnapshot.spiTxSentCnt,
		statsSnapshot.spiTxErrorCnt,
		statsSnapshot.spiRxPollCnt,
		statsSnapshot.spiRxCompleteCnt,
		statsSnapshot.spiRxEmptyRespCnt,
		statsSnapshot.spiRxNoRespFoundCnt,
		statsSnapshot.spiRxBusySkipCnt + statsSnapshot.spiRxStateSkipCnt,
		statsSnapshot.spiRxWatchdogResetCnt,
		stateName,
		statsSnapshot.spiRxCANQueuedCnt,
		statsSnapshot.spiRxCANQueueFailCnt,
		canTxQueuePercent, 
		streamState == STREAM_IDLE ? 0 : (streamState == KEYFRAME_ACTIVE ? 1 : 2), // Stream state indicator
		kfAckRxCnt,
		kfAckMissedCnt);

	if (len > 0 && len < (int)sizeof(buffer))
	{
		RAMN_UART_SendFromTask((uint8_t*)buffer, (uint32_t)len);
	}
#endif
}

// ============================================================================
// PERIODIC UPDATE FUNCTION
// ============================================================================
// Called periodically from main task (does not need to return quickly)
// Handles ESP32 polling state machine and periodic buffer flushing
// ============================================================================
void RAMN_TELEMATICS_Update(uint32_t tick)
{
	// ========================================================================
	// ESP32 POLLING STATE MACHINE (NON-BLOCKING)
	// ========================================================================
	// This state machine polls the ESP32 for pending CAN messages to transmit
	// All operations are non-blocking to maintain real-time CAN performance
	// ========================================================================

	// WATCHDOG: Check if stuck in COMPLETE state for too long (should process immediately)
	// This catches the case where Update() is being called but ProcessESP32Response() isn't running
	if (spiPollState == SPI_POLL_COMPLETE && (tick - spiPollCompleteTick) >= 50)
	{
		// CRITICAL FIX: Process the response BEFORE resetting state
		// Otherwise we skip response processing and the ESP32 queue backs up!
		ProcessESP32Response();

		// Clear the buffer to prevent stale data
		RAMN_memset(processRxBuffer, 0, SPI_RX_BUFFER_SIZE);

		// Now reset the state
		taskENTER_CRITICAL();
		spiStats.spiRxWatchdogResetCnt++;
		spiPollState = SPI_POLL_IDLE;
		taskEXIT_CRITICAL();
	}

	switch (spiPollState)
	{
		case SPI_POLL_IDLE:
			// Check if it's time to poll ESP32
			if ((tick - spiLastPollTick) >= currentPollIntervalMs)
			{
				if (RequestESP32Poll())
				{
					// Poll request successful
					// CRITICAL: Protect timestamp writes from interrupts (prevent word-tearing/corruption)
					taskENTER_CRITICAL();
					spiLastPollTick = tick;
					spiLastPollAttemptTick = tick;
					taskEXIT_CRITICAL();
					// State changed to SPI_POLL_REQUESTED by RequestESP32Poll
				}
				else
				{
					// Poll failed (bus busy) - retry sooner by updating attempt time
					// This allows faster retries when SPI becomes available
					taskENTER_CRITICAL();
					spiLastPollAttemptTick = tick;
					taskEXIT_CRITICAL();
					// Don't update spiLastPollTick - will retry next Update() call
				}
			}
			break;

		case SPI_POLL_REQUESTED:
			// Waiting for DMA completion (handled by callback)
			// Check for timeout
			if ((tick - spiPollRequestTick) >= SPI_POLL_TIMEOUT_MS)
			{
				// CRITICAL: Abort the ongoing DMA transfer to prevent SPI peripheral from getting stuck
				//extern SPI_HandleTypeDef hspi2;
				//HAL_SPI_Abort(&hspi2);

				// De-assert chip select (critical - must happen even if abort fails)
				HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

				taskENTER_CRITICAL();
				spiPollState = SPI_POLL_IDLE;
				spiOwner = SPI_OWNER_NONE;
				spiStats.spiRxTimeoutCnt++;
				taskEXIT_CRITICAL();
			}
			break;

		case SPI_POLL_COMPLETE:
			// Response received - process it
			ProcessESP32Response();

			// CRITICAL: Zero out the processed buffer to prevent stale data
			// Next poll will swap buffers, so we clear the one that will become activeRxBuffer
			RAMN_memset(processRxBuffer, 0, SPI_RX_BUFFER_SIZE);

			spiPollState = SPI_POLL_IDLE;
			break;

		case SPI_POLL_TIMEOUT:
			// Timeout occurred - reset and try again next interval

			// CRITICAL: Zero out the processed buffer to prevent stale data
			// Next poll will swap buffers, so we clear the one that will become activeRxBuffer
			RAMN_memset(processRxBuffer, 0, SPI_RX_BUFFER_SIZE);

			spiPollState = SPI_POLL_IDLE;
			break;
	}

	// Periodically flush SPI buffer to ensure messages don't get stuck
	// This handles low CAN traffic scenarios where buffer doesn't fill up
	// LOCK-FREE: Just read tick and call flush (no critical sections!)
	if ((tick - spiTxLastFlushTick) >= SPI_FLUSH_INTERVAL_MS)
	{
		FlushSPIBuffer();  // Non-blocking, DMA-based
		spiTxLastFlushTick = tick;
	}

	// ========================================================================
	// IMAGE STREAM STATE MANAGEMENT
	// ========================================================================

	// Report the last ACK from ECU A -- the only telemetry ECU A can produce.
	PrintImageACK();

	// KEYFRAME_SENT: wait for ACK from ECU A (0x303), timeout after 2 s
	if (streamState == KEYFRAME_SENT)
	{
		if (kfAckReceived == True)
		{
			kfAckReceived         = False;
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
		}
		else if ((tick - kfAckWaitTick) >= KF_ACK_TIMEOUT_MS)
		{
			// ACK timed out — give up and return to idle. Say so: silence here
			// and silence from a healthy ECU A look identical on the wire.
			PrintImageACKTimeout();
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
		}
	}

	// DELTA_ACTIVE: return to idle poll rate if no tile arrives within 2 s
	if (streamState == DELTA_ACTIVE)
	{
		if ((tick - lastDeltaActivityTick) >= DELTA_IDLE_TIMEOUT_MS)
		{
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
			deltaFirstTileOfFrame = False;
		}
	}

	// Periodically print SPI statistics to UART for monitoring
	if ((tick - spiStatsLastPrintTick) >= SPI_STATS_PRINT_INTERVAL_MS)
	{
		PrintSPIStats();
		spiStatsLastPrintTick = tick;
	}
}

// ============================================================================
// PUBLIC: STREAM ACTIVE QUERY
// ============================================================================
RAMN_Bool_t RAMN_TELEMATICS_IsStreamActive(void)
{
	return (streamState != STREAM_IDLE) ? True : False;
}

// ============================================================================
// PUBLIC: PROCESS IMG_ACK (0x303) FROM ECU A
// Called from RAMN_TELEMATICS_ProcessRxCANMessage when ID == IMG_CAN_ID_ACK.
// ============================================================================
void RAMN_TELEMATICS_ProcessImageACK(const FDCAN_RxHeaderTypeDef* pHeader,
                                     const uint8_t* data, uint32_t tick)
{
	(void)tick;

	// Stash the whole payload for the periodic task to print. ECU A has no UART
	// of its own -- ENABLE_UART is TARGET_ECUD only -- so this ACK is the only
	// thing it can say about a keyframe, and this function runs in the CAN RX
	// task, which is not where UART output belongs. Recorded unconditionally,
	// including an ACK that arrives in the wrong state, because that is itself
	// a diagnosis.
	uint8_t ackLen = DLCtoUINT8(pHeader->DataLength);
	if (ackLen > (uint8_t)sizeof(kfAckPayload)) ackLen = (uint8_t)sizeof(kfAckPayload);
	RAMN_memset(kfAckPayload, 0, sizeof(kfAckPayload));
	for (uint8_t k = 0U; k < ackLen; k++) kfAckPayload[k] = (data != NULL) ? data[k] : 0U;
	kfAckPayloadLen  = ackLen;
	kfAckPrintNeeded = True;
	kfAckRxCnt++;

	if (streamState != KEYFRAME_SENT) return;
	kfAckStatus   = (data != NULL) ? data[0] : 0xFFU;
	kfAckReceived = True;
}
