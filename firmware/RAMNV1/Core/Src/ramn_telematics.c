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
#ifdef ENABLE_IMAGE_SECOC
#include "ramn_secoc.h"
#include "ramn_secoc_keys.h"
#include "ramn_secoc_link.h"
#endif
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

// ============================================================================
// SPI TRANSACTION SIZE -- SHARED WITH THE ESP32
//
// This MUST equal SPI_BUFFER_SIZE in the add-on firmware's
// components/can_spi_protocol/include/can_spi_protocol.h. The ESP32 is the SPI
// slave and arms each transaction with trans.length = SPI_BUFFER_SIZE * 8;
// anything this master clocks beyond that is simply not captured.
//
// It was 160 here against 256 there for the poll direction (wasteful but safe),
// and 1024 here against 256 there for the flush direction -- so every CAN frame
// past the first ~256 bytes of a flush was clocked into nothing. That is the
// TX[Req:33600 Sent:7793] gap in the stats line.
//
// Throughput: one IMG_CHUNK SPI message is 68 bytes on the wire, and the poll
// now completes in one periodic tick, so chunks per transaction is the only
// remaining lever on this link -- and the link itself is nearly idle. At
// 32 MHz (SPI2, PCLK 128 MHz / 4) a 1020-byte transaction clocks in 255 us,
// 2.6% of a 10 ms tick.
//
// The size has to be a whole number of 68-byte messages or the remainder is
// clocked for nothing: 512 carried 7 and wasted 36 bytes on every transaction.
// 1020 is 15 of them exactly, and stays inside the 1024 the ESP32's DMA
// descriptor is sized for. That is 1500 chunks/s, about 26% of the CAN bus.
// [LEN][MARKER][TYPE][SEQ_HI][SEQ_LO][REAL_LEN] + payload + [CHK] = 6 + N + 1.
// The payload is whatever a 0x301 frame has room for once its CAN header --
// including the SecOC authenticator -- is subtracted, so this number follows
// the wire format automatically instead of being restated.
#define IMG_CHUNK_SPI_MSG_LEN (7U + IMG_CAN_CHUNK_PAYLOAD)

// SecOC shrinks the chunk payload from 61 to 57, which shrinks the SPI message
// from 68 bytes to 64 -- and 64 divides 1024 exactly. The transaction gets
// BIGGER and stops wasting the tail: 16 whole chunks per poll against 15, and
// zero remainder clocked for nothing, right at the 1024 the ESP32's DMA
// descriptor is sized for. The authenticator costs 7% of each frame's payload
// and hands most of it back in link efficiency.
#ifdef ENABLE_IMAGE_SECOC
#define SPI_TRANSACTION_SIZE 1024   // 16 x 64
#else
#define SPI_TRANSACTION_SIZE 1020   // 15 x 68
#endif

// Tile RLE bytes one 0x305 frame carries: 64 minus its header (and MAC).
#define DELTA_CAN_HEADER     DELTA_CAN_HEADER_BYTES
#define DELTA_CHUNK_PAYLOAD  DELTA_CAN_CHUNK_PAYLOAD

#define SPI_TX_BUFFER_SIZE SPI_TRANSACTION_SIZE  // one flush = one transaction

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
#define SPI_RX_BUFFER_SIZE SPI_TRANSACTION_SIZE   // whole IMG_CHUNKs per poll

/* One IMG_CHUNK SPI message is [LEN][MARKER][TYPE][SEQ_HI][SEQ_LO][REAL_LEN]
   + IMG_CAN_CHUNK_PAYLOAD + [CHK]: 64 bytes with SecOC on, 68 with it off.
   A transaction that cannot hold at least two of them is not worth the poll. */
_Static_assert(SPI_TRANSACTION_SIZE >= (2 * IMG_CHUNK_SPI_MSG_LEN),
               "SPI transaction too small to carry two image chunks");
_Static_assert(SPI_TRANSACTION_SIZE <= 1024,
               "raise the ESP32's SPI_BUFFER_SIZE to match before growing this");
/* Every byte past the last whole message is clocked and thrown away, so a size
   that is not a multiple of one is pure waste on every single transaction. */
_Static_assert((SPI_TRANSACTION_SIZE % IMG_CHUNK_SPI_MSG_LEN) == 0,
               "SPI transaction must be a whole number of IMG_CHUNK messages");
/* The ESP32's SPI slave DMA moves whole words. */
_Static_assert((SPI_TRANSACTION_SIZE % 4) == 0,
               "SPI transaction must be a multiple of 4 bytes");

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
// Measured IMG_END -> ACK round trip, printed by the periodic task.
static uint32_t    kfAckLatencyMs    = 0;
static RAMN_Bool_t kfAckLatencyPrint = False;
#define KF_ACK_TIMEOUT_MS 2000U
// Byte 0 of a 0x303 when ECU A saw IMG_END outside a keyframe. Mirrors
// IMG_ACK_LATE in ramn_screen_image.c.
#define IMG_ACK_LATE_STATUS 0x03U

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
static char spiStatsPrintBuf[288];
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
	volatile uint32_t spiRxRepeatRespCnt;  // Image transactions the ESP32 presented twice
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
static void PrintImageACKTimeout(uint32_t waited);
static void PrintImageACKLatency(void);

// ============================================================================
// INITIALIZATION
// ============================================================================
#ifdef ENABLE_IMAGE_SECOC
// ============================================================================
// SecOC -- SENDER SIDE
//
// One freshness domain covers the whole image stream. Keyframes and delta
// frames draw from the same counter, so a recorded delta frame cannot be
// replayed into the place of a later keyframe.
// ============================================================================
// The session -- key, freshness domain and handshake -- belongs to
// ramn_secoc_link.c, dispatched from main.c as a peer of this module's own
// ProcessRxCANMessage. It used to live here, which meant the frame builder and
// nonce draw existed a second time in ramn_screen_image.c for ECU A's half of
// the same exchange.

// Image messages from the ESP32 dropped because no session was up. Reported in
// the periodic SPI stats so a link that never handshakes is visible on UART
// rather than looking like an ESP32 that stopped sending.
static uint32_t             noSessionDrops = 0U;

// Freshness of the frame being sent right now. Every chunk of that frame is
// authenticated under it, which is exactly what lets a chunk spend zero wire
// bytes carrying freshness of its own -- only the frame's opening message
// (0x300 / 0x304) transmits it.
static uint32_t imgCurrentFv = 0U;

// Which session imgCurrentFv belongs to; see the note on ECU A's kfSessionGen.
static uint32_t imgSessionGen = 0U;

// Writes the authenticator into the LAST IMG_SECOC_MAC_BYTES of a frame that
// is dlcLen bytes on the wire, over everything before it.
//
// Anchoring the MAC to the end of the frame rather than to the end of the
// payload keeps its offset fixed no matter how many RLE bytes a chunk carries,
// and it pulls the whole header -- sequence number, length byte, tile
// coordinates -- inside the authenticated region. A receiver therefore cannot
// be steered by a tampered length field: changing one changes the MAC input.
static void SecOCTagFrame(uint32_t canId, uint8_t* data, uint8_t dlcLen, uint32_t fv)
{
	RAMN_SecOC_Ctx_t ctx;
	ctx.dataId = (uint16_t)canId;
	ctx.macLen = IMG_SECOC_MAC_BYTES;
	// The SESSION key, never the provisioned root. The root is spent once, on
	// the handshake in ramn_secoc_link.c, and does no per-frame work.
	ctx.key    = RAMN_SecOC_LINK_Key();
	ctx.fv     = RAMN_SecOC_LINK_Freshness();

	uint8_t authLen = (uint8_t)(dlcLen - IMG_SECOC_MAC_BYTES);
	RAMN_SecOC_ComputeMac(&ctx, fv, data, authLen, &data[authLen]);
}

// Places the truncated freshness value of a new frame at offset off, and
// returns the full value the frame's chunks will be authenticated under.
static uint32_t SecOCBeginFrame(uint8_t* data, uint8_t off)
{
	uint32_t fv = RAMN_SecOC_TxFreshness(RAMN_SecOC_LINK_Freshness());
	data[off]      = (uint8_t)((fv >> 8) & 0xFFU);
	data[off + 1U] = (uint8_t)( fv       & 0xFFU);
	return fv;
}

#endif

void RAMN_TELEMATICS_Init(uint32_t tick)
{
#ifdef ENABLE_IMAGE_SECOC
	// The session itself is initialised in main.c with the rest of the SecOC
	// modules; this only clears what belongs to the stream.
	imgCurrentFv = 0U;
#endif

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
	// Real time, to match the comparison in RAMN_TELEMATICS_Update -- see the
	// note there on why `tick` cannot be used for a deadline.
	lastDeltaActivityTick = (uint32_t)xTaskGetTickCount();

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

// A 32-bit FNV-1a over the whole transaction. Its own function, and the
// running value a static rather than a local, because ProcessESP32Response is
// the middle of the deepest chain on the periodic task's 1 KB stack and a
// single extra 32-bit local there costs 16 bytes of a 32-byte margin.
static uint32_t respFingerprint;
static uint32_t lastImageRespFingerprint;
static void FingerprintResponse(const uint8_t* buf)
{
	uint32_t h = 2166136261UL;
	for (uint16_t i = 0U; i < SPI_RX_BUFFER_SIZE; i++)
	{
		h ^= (uint32_t)buf[i];
		h *= 16777619UL;
	}
	respFingerprint = h;
}

static void ProcessESP32Response(void)
{
	uint8_t* rxBuf  = processRxBuffer;
	uint16_t offset = 0;

	// The ESP32 re-presents its staged SPI response whenever ECU D's write did
	// not parse as a poll, and on a full-duplex bus ECU D has already clocked
	// those bytes out. That is deliberate -- "re-present when unsure" is the
	// right behaviour for a link that cannot retransmit -- and it is only safe
	// because the receiver ignores what it has already seen.
	//
	// ECU A's chunk sequence gate does exactly that for 0x301, and its
	// duplicate counter proves the repeats are real: flags=0x10 on hardware.
	// But IMG_START and IMG_END carry no sequence of their own, and both change
	// state. Forwarded twice they wreck the frame they belong to:
	//
	//   IMG_START again  zeroes kfExpectedSeq and the counters mid-frame, and
	//                    queues a second START entry -- which ECU A's
	//                    latest-frame-wins skip then reads as "this frame is
	//                    stale", dropping a picture that was arriving fine.
	//   IMG_END again    finds imgState already IMG_SHOWN and comes back
	//                    IMG_ACK_LATE, and every chunk behind it is refused
	//                    out-of-state.
	//
	// Which is precisely the hardware signature: st=3 with decoded=0, then
	// flags=0x01/0x04/0x05 and a partial decode, all with drop=0/0 -- nothing
	// lost, everything delivered twice.
	//
	// Deduplicate here instead of gating each message type separately. This is
	// the one place that can see a whole transaction, and the repeat is a
	// property of the transaction, not of the messages inside it.
	FingerprintResponse(rxBuf);

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

	// Messages parsed out of one poll response.
	//
	// This was a hardcoded 2, sized to the old 160-byte transaction that held
	// exactly two image chunks. Growing the transaction to 512 -- seven chunks --
	// therefore changed nothing at all: ECU D still read the first two and threw
	// the other five away without a word. The bound has to follow the buffer, or
	// the buffer size is a number that only looks like it does something.
	//
	// The smallest possible message is 4 bytes ([LEN][0xCC][TYPE][CHK]), so this
	// is the real ceiling; the loop stops early as soon as no further marker is
	// found.
	for (int msgNum = 0; msgNum < (int)(SPI_TRANSACTION_SIZE / 4); msgNum++)
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

		// Only image traffic is deduplicated. A plain CAN frame relayed from
		// the ESP32 repeats identically all the time and legitimately -- a
		// periodic 100 ms frame polled faster than it changes is the normal
		// case -- so dropping those would be dropping real gateway traffic.
		// Image messages carry sequence numbers, so a byte-identical image
		// transaction can only be the same one twice.
		if ((msgNum == 0) && (msgType >= RAMN_MSG_TYPE_IMG_START))
		{
			if (respFingerprint == lastImageRespFingerprint)
			{
				spiStats.spiRxRepeatRespCnt++;
#ifdef TELEMATICS_SPI_DEBUG
				RAMN_UART_SendStringFromTask("SPI: repeat transaction skipped\r\n");
#endif
				return;
			}
			lastImageRespFingerprint = respFingerprint;
		}

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

#ifdef ENABLE_IMAGE_SECOC
		// FAIL CLOSED, sender side. Every image message from the ESP32 is
		// dropped until ECU A has confirmed a session, and each one nudges the
		// handshake along (rate limited inside). Dropping rather than queueing
		// is right: the ESP32 is sending live frames, so the next one is
		// always more useful than the one we held.
		if ((msgType == RAMN_MSG_TYPE_IMG_START) || (msgType == RAMN_MSG_TYPE_IMG_CHUNK) ||
		    (msgType == RAMN_MSG_TYPE_IMG_END)   || (msgType == RAMN_MSG_TYPE_IMG_ABORT) ||
		    (msgType == RAMN_MSG_TYPE_DELTA_FRAME) || (msgType == RAMN_MSG_TYPE_DELTA_FRAME_END))
		{
			if (RAMN_SecOC_LINK_EnsureSession(xTaskGetTickCount()) == 0U)
			{
				noSessionDrops++;
				continue;
			}
			// A new session restarts its freshness counters, so the frame
			// freshness cached from the previous one must go with it.
			{
				uint32_t gen = RAMN_SecOC_LINK_Generation();
				if (gen != imgSessionGen) { imgSessionGen = gen; imgCurrentFv = 0U; }
			}
		}
#endif

		// ------------------------------------------------------------------
		// TYPE 0x81: IMG_START — start of a keyframe
		// Format: [LEN][0xCC][0x81][W_HI][W_LO][H_HI][H_LO][CH_HI][CH_LO][X_OFF][Y_OFF][SCALE][CHK]
		//
		// SCALE is how many times ECU A repeats each source pixel in both axes.
		// It is the last field so that a sender predating it still parses: a
		// 12-byte IMG_START has no byte 11, and 0 reaches ECU A as 1:1.
		// ------------------------------------------------------------------
		if (msgType == RAMN_MSG_TYPE_IMG_START)
		{
			if (msgLen < 10U) { spiStats.spiRxInvalidCnt++; continue; }

			uint16_t w  = (uint16_t)((uint16_t)msg[4] | ((uint16_t)msg[3] << 8));
			uint16_t h  = (uint16_t)((uint16_t)msg[6] | ((uint16_t)msg[5] << 8));
			uint16_t ch = (uint16_t)((uint16_t)msg[8] | ((uint16_t)msg[7] << 8));
			uint8_t  xo = msg[9];
			uint8_t  yo = msg[10];
			uint8_t  sc = (msgLen >= 11U) ? msg[11] : 0U;

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

			// Forward 0x300 IMG_START to ECU A.
			// With SecOC the frame grows to 20 bytes: the 12-byte body is
			// untouched, the frame's truncated freshness follows it, and the
			// authenticator sits in the last four. IMG_START is one frame per
			// keyframe, so the two padding bytes cost nothing worth reclaiming.
			uint8_t canData[20];
			canData[0]  = (uint8_t)(w & 0xFFU);
			canData[1]  = (uint8_t)(w >> 8);
			canData[2]  = (uint8_t)(h & 0xFFU);
			canData[3]  = (uint8_t)(h >> 8);
			canData[4]  = (uint8_t)(ch & 0xFFU);
			canData[5]  = (uint8_t)(ch >> 8);
			canData[6]  = xo;
			canData[7]  = yo;
			canData[8]  = sc;      // pixel repeat factor, 0 or 1 = as-is
			canData[9]  = 0x00U;
			canData[10] = 0x01U;   // VERSION
			uint8_t xorChk = 0U;
			for (uint8_t k = 0U; k < 11U; k++) xorChk ^= canData[k];
			canData[11] = xorChk;
#ifdef ENABLE_IMAGE_SECOC
			// A new frame, so a new freshness value -- and every chunk and the
			// IMG_END behind it are authenticated under this same value.
			canData[14] = 0x00U;
			canData[15] = 0x00U;
			imgCurrentFv = SecOCBeginFrame(canData, 12U);
			SecOCTagFrame(IMG_CAN_ID_START, canData, 20U, imgCurrentFv);
			SendImageCANFrame(IMG_CAN_ID_START, FDCAN_DLC_BYTES_20, False, canData);
#else
			SendImageCANFrame(IMG_CAN_ID_START, FDCAN_DLC_BYTES_12, False, canData);
#endif
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
				uint8_t frameLen  = (remaining <= IMG_CAN_CHUNK_PAYLOAD)
				                    ? remaining : (uint8_t)IMG_CAN_CHUNK_PAYLOAD;

				uint8_t canData[64];
				RAMN_memset(canData, 0, sizeof(canData));
				canData[0] = msg[3];    // SEQ_HI
				canData[1] = msg[4];    // SEQ_LO
				canData[2] = frameLen;  // REAL_LEN -- true byte count carried in this frame
				for (uint8_t k = 0U; k < frameLen; k++) canData[3U + k] = msg[6U + srcOffset + k];
#ifdef ENABLE_IMAGE_SECOC
				// Under the freshness of the keyframe this chunk belongs to.
				// The sequence number is inside the authenticated region, so a
				// chunk cannot be lifted out of one keyframe and replayed into
				// another position of the same one.
				SecOCTagFrame(IMG_CAN_ID_DATA, canData, 64U, imgCurrentFv);
#endif
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
			uint8_t canData[12];
			RAMN_memset(canData, 0, sizeof(canData));
			canData[0] = (uint8_t)(kfChunksSent & 0xFFU);
			canData[1] = (uint8_t)(kfChunksSent >> 8);
			// CRC16 bytes (2-3) left as 0x00 — full CRC computation is optional
			canData[4] = 0x00U;   // STATUS OK
#ifdef ENABLE_IMAGE_SECOC
			// Same freshness as the keyframe it closes: IMG_END is part of
			// that frame, not a frame of its own.
			SecOCTagFrame(IMG_CAN_ID_END, canData, 12U, imgCurrentFv);
			SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_12, False, canData);
#else
			SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_8, False, canData);
#endif

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
				uint8_t canData[12];
				RAMN_memset(canData, 0, sizeof(canData));
				canData[4] = 0x01U;   // STATUS abort
#ifdef ENABLE_IMAGE_SECOC
				SecOCTagFrame(IMG_CAN_ID_END, canData, 12U, imgCurrentFv);
				SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_12, False, canData);
#else
				SendImageCANFrame(IMG_CAN_ID_END, FDCAN_DLC_BYTES_8, False, canData);
#endif
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

				uint8_t startData[16];
				RAMN_memset(startData, 0, sizeof(startData));
				startData[0] = deltaFrameSeq;
				startData[1] = 0U;   // tile_count filled in DELTA_FRAME_END
#ifdef ENABLE_IMAGE_SECOC
				// A delta frame is a frame: it takes its own freshness, and
				// its tiles inherit it. This message MUST be authenticated --
				// it is what tells ECU A which freshness the tiles behind it
				// are signed under, so an unauthenticated one would let an
				// attacker choose that value.
				imgCurrentFv = SecOCBeginFrame(startData, 8U);
				SecOCTagFrame(DELTA_CAN_ID_FRAME_START, startData, 16U, imgCurrentFv);
				SendImageCANFrame(DELTA_CAN_ID_FRAME_START, FDCAN_DLC_BYTES_16, False, startData);
#else
				SendImageCANFrame(DELTA_CAN_ID_FRAME_START, FDCAN_DLC_BYTES_8, False, startData);
#endif

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
			// 64 minus the 5-byte [X][Y][SIZE][SEQ][LEN] header above. The
			// encoder must cut its tile chunks at exactly this, because
			// anything longer is truncated right here without a word.
			// RAMN_DELTA_CHUNK_PAYLOAD in the vendored vectors is the same
			// number, and the host tests assert they agree.
			uint8_t copyLen = (payLen <= DELTA_CHUNK_PAYLOAD) ? payLen : DELTA_CHUNK_PAYLOAD;
			canData[4] = copyLen;
			for (uint8_t k = 0U; k < copyLen; k++) canData[5U + k] = msg[8U + k];
#ifdef ENABLE_IMAGE_SECOC
			// Tile coordinates and size are inside the authenticated region,
			// so a valid tile cannot be relocated somewhere else on the panel.
			SecOCTagFrame(DELTA_CAN_ID_TILE_CHUNK, canData, 64U, imgCurrentFv);
#endif
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

#ifdef ENABLE_IMAGE_SECOC
	// The handshake messages are RAMN_SecOC_LINK_ProcessRxCANMessage's, called
	// from main.c beside this function. They are still swallowed here so they
	// are not forwarded to the ESP32: the session is between the two STM32s
	// and the add-on board has no part in it.
	if ((pHeader->IdType == FDCAN_STANDARD_ID) &&
	    (pHeader->Identifier >= SESSION_CAN_ID_REQ) &&
	    (pHeader->Identifier <= SESSION_CAN_ID_CONFIRM)) return;
#endif

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
		    "ECUA ACK: st=%u flags=0x%02X decoded=%lu/115200 rx=%u ringdrop=%u canovr=%u (sent %u)\r\n",
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

// `waited` is the MEASURED wait, not KF_ACK_TIMEOUT_MS. Printing the constant
// says only "the branch that fires after 2 s fired", which is exactly what a
// mismatched-clock comparison also prints while firing immediately -- the two
// are indistinguishable in a log, and telling them apart is the whole question
// when a keyframe looks slow.
// Its own function, like PrintImageACK next door and for the same reason: the
// snprintf locals belong to whoever calls it, and RAMN_TELEMATICS_Update is the
// root of the deepest chain on the periodic task's 1 KB stack. Inlined here it
// took that chain to 688 of its 704-byte budget -- the margin that has bricked
// this ECU before.
static void PrintImageACKLatency(void)
{
#ifdef ENABLE_UART
	if (kfAckLatencyPrint == False) return;
	kfAckLatencyPrint = False;
	int len = snprintf(imgAckPrintBuf, sizeof(imgAckPrintBuf),
	    "ECUA ACK: answered in %lums\r\n", (unsigned long)kfAckLatencyMs);
	if (len > 0) RAMN_UART_SendFromTask((uint8_t*)imgAckPrintBuf, (uint32_t)len);
#endif
}

static void PrintImageACKTimeout(uint32_t waited)
{
#ifdef ENABLE_UART
	int len = snprintf(imgAckPrintBuf, sizeof(imgAckPrintBuf),
	    "ECUA ACK: TIMEOUT after %lums (limit %ums, sent %u chunks)\r\n",
	    (unsigned long)waited, (unsigned)KF_ACK_TIMEOUT_MS, kfChunksSent);
	if (len > 0) RAMN_UART_SendFromTask((uint8_t*)imgAckPrintBuf, (uint32_t)len);
#endif
}

static void PrintSPIStats(void)
{
#ifdef ENABLE_UART
	// Static for the same reason as imgAckPrintBuf: this is the deepest UART
	// frame on the periodic task's 1 KB stack, and the stats line only grows.
	char *buffer = spiStatsPrintBuf;
	const size_t bufferSize = sizeof(spiStatsPrintBuf);
	int len;

	// Capture stats atomically to prevent corruption during printing
	taskENTER_CRITICAL();
	RAMN_SPI_Stats_t statsSnapshot = spiStats;
	SPI_PollState_t currentState = spiPollState;
	taskEXIT_CRITICAL();

	// FDCAN bus health. The image data frames (0x301) are the only traffic on
	// this bus that uses BRS, so DLEC -- the DATA-phase error code -- is the
	// direct answer to whether the fast phase works. LEC covers the arbitration
	// phase. Both self-clear to 7 (NO_CHANGE) once read, so a nonzero value
	// means an error since the last stats line, not a stale one.
	FDCAN_ProtocolStatusTypeDef ps;
	uint8_t lec = 7U, dlec = 7U, busoff = 0U, errpass = 0U;
	if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) == HAL_OK)
	{
		lec     = (uint8_t)ps.LastErrorCode;
		dlec    = (uint8_t)ps.DataLastErrorCode;
		busoff  = (uint8_t)ps.BusOff;
		errpass = (uint8_t)ps.ErrorPassive;
	}
	FDCAN_ErrorCountersTypeDef ec;
	uint8_t tec = 0U, rec = 0U;
	if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) == HAL_OK)
	{
		tec = (uint8_t)ec.TxErrorCnt;
		rec = (uint8_t)ec.RxErrorCnt;
	}

	// Get CAN TX queue usage
	size_t canTxQueueUsed = xStreamBufferBytesAvailable(CANTxDataStreamBufferHandle);
	size_t canTxQueueFree = xStreamBufferSpacesAvailable(CANTxDataStreamBufferHandle);
	size_t canTxQueueTotal = canTxQueueUsed + canTxQueueFree;
	uint8_t canTxQueuePercent = (canTxQueueTotal > 0) ? ((canTxQueueUsed * 100) / canTxQueueTotal) : 0;

	// State names for debugging
	const char* stateNames[] = {"IDLE", "REQ", "DONE", "TO"};
	const char* stateName = (currentState <= SPI_POLL_TIMEOUT) ? stateNames[currentState] : "UNK";

	// Print compact stats on single line to reduce UART load
	len = snprintf(buffer, bufferSize,
		"SPI: TX[Req:%lu Sent:%lu Err:%lu] RX[Poll:%lu OK:%lu Empty:%lu NoResp:%lu Rpt:%lu Skip:%lu WD:%lu St:%s Q:%lu QFail:%lu] CANTxQ:%u%%  StreamState:%u ECUAack:%lu miss:%lu BUS[TEC:%u REC:%u LEC:%u DLEC:%u BO:%u EP:%u RxOvr:%lu]\r\n",
		statsSnapshot.spiTxRequestCnt,
		statsSnapshot.spiTxSentCnt,
		statsSnapshot.spiTxErrorCnt,
		statsSnapshot.spiRxPollCnt,
		statsSnapshot.spiRxCompleteCnt,
		statsSnapshot.spiRxEmptyRespCnt,
		statsSnapshot.spiRxNoRespFoundCnt,
		statsSnapshot.spiRxRepeatRespCnt,
		statsSnapshot.spiRxBusySkipCnt + statsSnapshot.spiRxStateSkipCnt,
		statsSnapshot.spiRxWatchdogResetCnt,
		stateName,
		statsSnapshot.spiRxCANQueuedCnt,
		statsSnapshot.spiRxCANQueueFailCnt,
		canTxQueuePercent, 
		streamState == STREAM_IDLE ? 0 : (streamState == KEYFRAME_ACTIVE ? 1 : 2), // Stream state indicator
		kfAckRxCnt,
		kfAckMissedCnt,
		tec, rec, lec, dlec, busoff, errpass,
		RAMN_FDCAN_Status.CANRxOverrunCnt);

	if (len > 0 && len < (int)bufferSize)
	{
		RAMN_UART_SendFromTask((uint8_t*)buffer, (uint32_t)len);
	}
#endif
}

// The keyframe-ACK wait and the delta idle timeout, in their own function so
// that the tick they need is not a local of RAMN_TELEMATICS_Update: that is the
// root of the deepest chain on the periodic task's 1 KB stack, and one extra
// 32-bit local there took it from 672 to 688 of its 704-byte budget -- the
// margin that has bricked this ECU before.
static void UpdateStreamTimeouts(void)
{
	uint32_t now = (uint32_t)xTaskGetTickCount();

	// KEYFRAME_SENT: wait for ACK from ECU A (0x303), timeout after 2 s
	if (streamState == KEYFRAME_SENT)
	{
		if (kfAckReceived == True)
		{
			// How long ECU A took from IMG_END to answering. That is its whole
			// decode-and-paint time for the frame, and it is the number that
			// decides how fast keyframes can be sent -- worth one print.
			kfAckLatencyMs        = (uint32_t)(now - kfAckWaitTick);
			kfAckLatencyPrint     = True;
			kfAckReceived         = False;
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
		}
		else if ((int32_t)(now - kfAckWaitTick) >= (int32_t)KF_ACK_TIMEOUT_MS)
		{
			// ACK timed out — give up and return to idle. Say so: silence here
			// and silence from a healthy ECU A look identical on the wire.
			PrintImageACKTimeout((uint32_t)(now - kfAckWaitTick));
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
#ifdef ENABLE_IMAGE_SECOC
			// Most likely cause of a silent ECU A is that it rebooted and no
			// longer holds the session key we are streaming under. It cannot
			// tell us so -- anything it sent would itself need a session -- so
			// the timeout is the signal. Drop the session and the next image
			// message re-handshakes.
			RAMN_SecOC_LINK_Drop();
#endif
		}
	}

	// DELTA_ACTIVE: return to idle poll rate if no tile arrives within 2 s
	if (streamState == DELTA_ACTIVE)
	{
		if ((int32_t)(now - lastDeltaActivityTick) >= (int32_t)DELTA_IDLE_TIMEOUT_MS)
		{
			streamState           = STREAM_IDLE;
			currentPollIntervalMs = SPI_POLL_INTERVAL_MS;
			deltaFirstTileOfFrame = False;
		}
	}
}

// ============================================================================
// PERIODIC UPDATE FUNCTION
// ============================================================================
// Called periodically from main task (does not need to return quickly)
// Handles ESP32 polling state machine and periodic buffer flushing
// ============================================================================
void RAMN_TELEMATICS_Update(uint32_t tick)
{
	// `tick` is the periodic task's xLastWakeTime. vTaskDelayUntil advances it
	// by exactly SIM_LOOP_CLOCK_MS per iteration, so whenever this loop
	// overruns its period it falls behind real time and never catches up --
	// the same defect that used to tear down ECU A's image screen mid-keyframe.
	//
	// Four timeouts below are stamped with xTaskGetTickCount() (real time) in
	// ProcessESP32Response and RequestESP32Poll, and were then compared against
	// `tick`. Once the two diverge that subtraction is negative, wraps to about
	// 4.29 billion, and every one of those timeouts fires on the first call
	// after it is armed: the keyframe ACK wait ends instantly and drops the
	// poll interval from 1 ms back to 50 ms, the delta stream is torn down
	// between tiles, and the poll watchdogs trip on healthy polls.
	//
	// Read the clock those stamps were taken from, and compare as a SIGNED
	// difference so a lagging or wrapped tick can never fire one early.

	// ========================================================================
	// ESP32 POLLING STATE MACHINE (NON-BLOCKING)
	// ========================================================================
	// This state machine polls the ESP32 for pending CAN messages to transmit
	// All operations are non-blocking to maintain real-time CAN performance
	// ========================================================================

	// WATCHDOG: Check if stuck in COMPLETE state for too long (should process immediately)
	// This catches the case where Update() is being called but ProcessESP32Response() isn't running
	if (spiPollState == SPI_POLL_COMPLETE &&
	    ((int32_t)((uint32_t)xTaskGetTickCount() - spiPollCompleteTick) >= 50))
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

	// The poll costs one state transition per call: request on one tick, notice
	// the DMA finished and process on the next, and only on a third is the
	// machine back in IDLE to ask again. At a 10 ms period that is ~30 ms a
	// poll however small currentPollIntervalMs is set -- measured at 32.4 ms
	// while streaming, with the interval already down at 1 ms. Seven chunks a
	// poll made that 216 chunks/s, so a 1120-chunk keyframe of a photograph
	// took 5.2 seconds.
	//
	// The DMA for a 512-byte transaction finishes in well under a tick, so the
	// wait was never for the hardware -- it was for the next call. Letting the
	// machine take as many steps as it can without blocking collapses the
	// round trip to one tick: process the response and ask again in the same
	// call. Two passes is all it can ever use, since a fresh request leaves
	// REQUESTED and there is nothing to do but wait for the interrupt.
	switch (spiPollState)
	{
		case SPI_POLL_IDLE:
			break;   // nothing outstanding; a poll starts in the request below

		case SPI_POLL_REQUESTED:
			// Waiting for DMA completion (handled by callback)
			// Check for timeout
			if ((int32_t)((uint32_t)xTaskGetTickCount() - spiPollRequestTick) >=
			    (int32_t)SPI_POLL_TIMEOUT_MS)
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

	// Ask again in the SAME call, now that whatever the last one left has been
	// dealt with. Deferring this to the next tick is what made a poll cost
	// three of them however small currentPollIntervalMs was set.
	if ((spiPollState == SPI_POLL_IDLE) && ((tick - spiLastPollTick) >= currentPollIntervalMs))
	{
		if (RequestESP32Poll())
		{
			// CRITICAL: Protect timestamp writes from interrupts (prevent word-tearing/corruption)
			taskENTER_CRITICAL();
			spiLastPollTick = tick;
			spiLastPollAttemptTick = tick;
			taskEXIT_CRITICAL();
			// State changed to SPI_POLL_REQUESTED by RequestESP32Poll
		}
		else
		{
			// Bus busy. spiLastPollTick is deliberately not advanced, so the
			// next call retries immediately rather than waiting an interval.
			taskENTER_CRITICAL();
			spiLastPollAttemptTick = tick;
			taskEXIT_CRITICAL();
		}
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
	PrintImageACKLatency();

	UpdateStreamTimeouts();

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

	// IMG_ACK_LATE means "0x302 reached me but I was not receiving a keyframe"
	// -- the opposite of a completion. Taken as one it ends the wait early and
	// reports its arrival as the frame's paint time, which is why a log full of
	// genuine 70-90 ms paints was salted with 7-12 ms ones that measured
	// nothing. Let the real ACK, or the timeout, end the wait.
	if (((data != NULL) ? data[0] : 0xFFU) == IMG_ACK_LATE_STATUS) return;

	kfAckStatus   = (data != NULL) ? data[0] : 0xFFU;
	kfAckReceived = True;
}
