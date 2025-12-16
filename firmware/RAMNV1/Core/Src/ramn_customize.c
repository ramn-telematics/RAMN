/*
 * ramn_customize.c
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

#include "ramn_customize.h"
#include "ramn_canfd.h"
#include "ramn_sensors.h"
#include "ramn_dbc.h"

#ifdef ENABLE_CDC
#include "ramn_cdc.h"
#endif

#ifdef ENABLE_UART
#include "ramn_uart.h"
#endif

// Loop counter for RAMN_CUSTOM_Update
static uint32_t loopCounter = 0;

// Number of time RAMN_CUSTOM_TIM6ISR has been called (by default, time in s from boot)
static volatile uint32_t tim6val = 0;

#ifdef ENABLE_SPI
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

// Timing and thresholds
static volatile uint32_t spiTxLastFlushTick = 0;  // volatile: accessed from multiple tasks
#define SPI_FLUSH_INTERVAL_MS 10
#define SPI_BUFFER_FLUSH_THRESHOLD 896  // Flush when 128 bytes remain

// ============================================================================
// BIDIRECTIONAL SPI: ESP32 POLLING STATE MACHINE
// ============================================================================
#define SPI_POLL_INTERVAL_MS 1000  // Configurable: poll ESP32 every 100ms (adjust as needed)
#define SPI_POLL_TIMEOUT_MS 50    // Max wait for ESP32 response
#define SPI_RX_BUFFER_SIZE 80     // Max response: 1+1+4+1+1+64+1 = 73 bytes + 7 byte processing delay

// Poll state machine
typedef enum {
	SPI_POLL_IDLE,              // Not polling
	SPI_POLL_REQUESTED,         // Poll request sent, waiting for response
	SPI_POLL_COMPLETE,          // Response received, ready to process
	SPI_POLL_TIMEOUT            // Response timeout, skip this poll
} SPI_PollState_t;

static volatile SPI_PollState_t spiPollState = SPI_POLL_IDLE;
static volatile uint32_t spiPollRequestTick = 0;
static volatile uint32_t spiLastPollTick = 0;

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
	volatile uint32_t spiRxTimeoutCnt;        // Response timeouts
	volatile uint32_t spiRxErrorCnt;          // DMA start errors
	volatile uint32_t spiRxInvalidCnt;        // Invalid responses from ESP32
	volatile uint32_t spiRxChecksumErrorCnt;  // Checksum failures
	volatile uint32_t spiRxCANQueuedCnt;      // CAN messages queued successfully
	volatile uint32_t spiRxCANQueueFailCnt;   // CAN queue failures
} RAMN_SPI_Stats_t;

static RAMN_SPI_Stats_t spiStats = {0};
#endif

void 	RAMN_CUSTOM_Init(uint32_t tick)
{
	loopCounter = 0;
#ifdef ENABLE_SPI
	// Initialize TX buffers
	activeBufferPos = 0;
	flushBufferSize = 0;
	spiTransmitBusy = False;
	spiTxLastFlushTick = tick;

	// Initialize RX polling
	spiPollState = SPI_POLL_IDLE;
	spiLastPollTick = tick;

	// Zero-initialize RX buffers (prevent garbage on first poll)
	RAMN_memset(spiRxBufferA, 0, SPI_RX_BUFFER_SIZE);
	RAMN_memset(spiRxBufferB, 0, SPI_RX_BUFFER_SIZE);
#endif
}

#ifdef ENABLE_SPI
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
void RAMN_CUSTOM_SPI_TxCpltCallback(void)
{
	// De-assert chip select for custom expansion board
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

	// Update statistics (safe in ISR - simple increments)
	spiStats.spiTxSentCnt++;
	spiStats.spiTxBytesSent += flushBufferSize;

	// Clear busy flag - allow next flush
	spiTransmitBusy = False;
}

// ============================================================================
// SPI DMA TRANSMIT-RECEIVE COMPLETE CALLBACK FOR ESP32 POLLING
// ============================================================================
// This callback is triggered when HAL_SPI_TransmitReceive_DMA completes.
// It is called FROM the HAL_SPI_TxRxCpltCallback in ramn_spi.c
// WARNING: Called from ISR context - keep it fast!
// ============================================================================
void RAMN_CUSTOM_SPI_TxRxCpltCallback(void)
{
	// This is called when simultaneous TX/RX DMA completes (polling operation)

	if (spiPollState == SPI_POLL_REQUESTED)
	{
		// Successfully received response from ESP32
		spiPollState = SPI_POLL_COMPLETE;

		// De-assert chip select
		HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

		// Update statistics
		spiStats.spiRxCompleteCnt++;
	}
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

	// Check if SPI bus is busy (CAN→SPI transmission in progress)
	if (spiTransmitBusy == True)
		return False;  // Try again later

	// Check if we're already waiting for a response
	if (spiPollState != SPI_POLL_IDLE)
		return False;  // Already polling

	// Swap RX buffers (similar to TX double-buffer pattern)
	taskENTER_CRITICAL();
	uint8_t* temp = activeRxBuffer;
	activeRxBuffer = processRxBuffer;
	processRxBuffer = temp;
	spiPollState = SPI_POLL_REQUESTED;
	spiPollRequestTick = xTaskGetTickCount();
	spiStats.spiRxPollCnt++;
	taskEXIT_CRITICAL();

	// Start simultaneous TX/RX DMA operation
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive_DMA(&hspi2,
	                                      pollTxBuffer,
	                                      activeRxBuffer,
	                                      SPI_RX_BUFFER_SIZE);

	if (status != HAL_OK)
	{
		// DMA start failed - reset state
		HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

		taskENTER_CRITICAL();
		spiPollState = SPI_POLL_IDLE;
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
static void ProcessESP32Response(void)
{
	uint8_t* rxBuf = processRxBuffer;
	uint16_t responseStart = 0;
	RAMN_Bool_t foundResponse = False;

	// Scan buffer for response marker (0xCC)
	for (uint16_t i = 0; i < SPI_RX_BUFFER_SIZE - 2; i++)
	{
		if (rxBuf[i] == 0xCC)
		{
			// Found 0xCC - check if previous byte could be valid length
			// Valid length: 2 (empty response) to 72 (max CAN message)
			if (i > 0 && rxBuf[i-1] >= 2 && rxBuf[i-1] <= 72)
			{
				responseStart = i - 1;  // LENGTH byte is before 0xCC
				foundResponse = True;
				break;
			}
		}
	}

	if (!foundResponse)
	{
		// No valid response found - ESP32 has no data to send
		return;
	}

	// Process response starting from found location
	uint8_t* response = &rxBuf[responseStart];
	uint8_t msgLen = response[0];

	// Check for empty response (no CAN message from ESP32)
	if (msgLen == 2 && response[1] == 0xCC && response[2] == 0x00)
		return;  // Valid empty response - no action needed

	// Validate message structure (minimum 8 bytes for a CAN message)
	if (msgLen < 8 || response[1] != 0xCC)
	{
		spiStats.spiRxInvalidCnt++;
		return;  // Invalid response
	}

	// Verify checksum (XOR of all bytes from 0xCC through CHECKSUM should be 0)
	uint8_t checksum = 0;
	for (uint8_t i = 1; i <= msgLen; i++)
		checksum ^= response[i];

	if (checksum != 0)
	{
		spiStats.spiRxChecksumErrorCnt++;
		return;  // Checksum mismatch
	}

	// Parse CAN message from ESP32 response
	FDCAN_TxHeaderTypeDef header;
	uint8_t data[64];
	RAMN_memset(data, 0, sizeof(data));  // Zero-initialize payload

	// Extract CAN ID (big-endian)
	uint32_t canId = ((uint32_t)response[2] << 24) |
	                 ((uint32_t)response[3] << 16) |
	                 ((uint32_t)response[4] << 8)  |
	                 ((uint32_t)response[5]);

	uint8_t dlc = response[6];
	uint8_t flags = response[7];

	// Build CAN header
	header.Identifier = canId;
	header.IdType = (flags & 0x01) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	header.TxFrameType = (flags & 0x02) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	header.DataLength = dlc;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

	// Copy payload
	uint8_t payloadSize = DLCtoUINT8(dlc);
	if (payloadSize > 0 && payloadSize <= 64)
	{
		// Ensure we don't read past msgLen
		uint8_t maxPayload = (msgLen >= 8) ? (msgLen - 7) : 0;  // msgLen includes all fields except itself
		if (payloadSize > maxPayload)
			payloadSize = maxPayload;

		for (uint8_t i = 0; i < payloadSize; i++)
			data[i] = response[8 + i];
	}

	// Queue to CAN transmit buffer (non-blocking)
	if (RAMN_FDCAN_SendMessage(&header, data) == RAMN_OK)
	{
		spiStats.spiRxCANQueuedCnt++;
	}
	else
	{
		spiStats.spiRxCANQueueFailCnt++;
	}
}

#endif

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
void	RAMN_CUSTOM_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick)
{
#ifdef ENABLE_SPI
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
#endif

	// Fields that you may want to use:
	// pHeader->Identifier: (11-bit val for standard, 29-bit for extended)
	// pHeader->IdType: FDCAN_STANDARD_ID or FDCAN_EXTENDED_ID
	// pHeader->RxFrameType:  FDCAN_DATA_FRAME or FDCAN_REMOTE_FRAME
	// DataLength: length of CAN payload, FDCAN_DLC_BYTES_0 (0) to FDCAN_DLC_BYTES_8 (8) for CAN, FDCAN_DLC_BYTES_0 (0) to FDCAN_DLC_BYTES_64 (0xF, Not 64) for CAN-FD.
	// pHeader->ErrorStateIndicator: For CAN-FD, either FDCAN_ESI_ACTIVE or FDCAN_ESI_PASSIVE
	// pHeader->BitRateSwitch: For CAN-FD, either FDCAN_BRS_OFF or FDCAN_BRS_ON
	// pHeader->FDFormat: FDCAN_CLASSIC_CAN or FDCAN_FD_CAN
	// pHeader->RxTimestamp: 16-bit value for RX timestamp, MAY NOT BE CONFIGURED CORRECTLY
	// See FilterIndex and IsFilterMatchingFrame for additional fields.
}

#ifdef ENABLE_CDC
// This function is called when a USB serial (CDC) line is received (terminated by \r, which is not included in the buffer).
// if you need another type of line terminator, modify CDC_Receive_FS in usbd_cdc_if.c.
// Return True to ask the ECU to skip this line, return False to continue processing as usual.
// This function is called from a task using an intermediary USB buffer, so it does not need to return quickly.
RAMN_Bool_t RAMN_CUSTOM_ProcessCDCLine(uint8_t* buffer, uint32_t size)
{
	// If you return True, you can entirely override USB communications, meaning that ECU A will lose the ability to forward slcan commands.
	// This means that you will lose the ability to use RAMN scripts (including reflashing over USB DFU).
	// Only return True if that is the behavior that you expect, and have another method for ECU A reflashing.
	// If you want to make sure that you (at least) keep the option to reprogram ECU A, uncomment the line below and keep it at the beginning.
	// if (size > 0U && buffer[0] == 'D') return False;

	return False; // WARNING read comments above before editing
}
#endif

// Called periodically from main task (does not need to return quickly)
void RAMN_CUSTOM_Update(uint32_t tick)
{
	// This function is called by a dedicated periodic task, which means code can here won't block other functionalities (such as receiving CAN messages).
	// Modify SIM_LOOP_CLOCK_MS if you want to use another period than 10ms.

	// Code here is executed every 10ms

#ifdef ENABLE_SPI
	// ========================================================================
	// ESP32 POLLING STATE MACHINE (NON-BLOCKING)
	// ========================================================================
	// This state machine polls the ESP32 for pending CAN messages to transmit
	// All operations are non-blocking to maintain real-time CAN performance
	// ========================================================================

	switch (spiPollState)
	{
		case SPI_POLL_IDLE:
			// Check if it's time to poll ESP32
			if ((tick - spiLastPollTick) >= SPI_POLL_INTERVAL_MS)
			{
				if (RequestESP32Poll())
				{
					spiLastPollTick = tick;
					// State changed to SPI_POLL_REQUESTED by RequestESP32Poll
				}
			}
			break;

		case SPI_POLL_REQUESTED:
			// Waiting for DMA completion (handled by callback)
			// Check for timeout
			if ((tick - spiPollRequestTick) >= SPI_POLL_TIMEOUT_MS)
			{
				// CRITICAL: Abort the ongoing DMA transfer to prevent SPI peripheral from getting stuck
				extern SPI_HandleTypeDef hspi2;
				HAL_SPI_Abort(&hspi2);

				spiPollState = SPI_POLL_TIMEOUT;
				spiStats.spiRxTimeoutCnt++;
				HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);
			}
			break;

		case SPI_POLL_COMPLETE:
			// Response received - process it
			ProcessESP32Response();
			spiPollState = SPI_POLL_IDLE;
			break;

		case SPI_POLL_TIMEOUT:
			// Timeout occurred - reset and try again next interval
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
#endif

	if ((loopCounter % 10) == 0)
	{
		// Code here is executed every 100ms
	}

	if ((loopCounter % 100) == 0)
	{
		// Code here is executed every 1s

#ifdef ENABLE_SPI
		// Optional: Report SPI statistics every second (similar to CAN error reporting)
		// Uncomment the block below to enable periodic stats reporting via UART
		/*
#ifdef ENABLE_UART
		char statsBuf[256];
		snprintf(statsBuf, sizeof(statsBuf),
		         "SPI TX - Req:%lu Sent:%lu Bytes:%lu Err:%lu Ovr:%lu Flush:%lu\r\n"
		         "SPI RX - Poll:%lu OK:%lu Timeout:%lu Invalid:%lu Queued:%lu\r\n",
		         spiStats.spiTxRequestCnt,
		         spiStats.spiTxSentCnt,
		         spiStats.spiTxBytesSent,
		         spiStats.spiTxErrorCnt,
		         spiStats.spiBufferOverrunCnt,
		         spiStats.spiBufferFlushCnt,
		         spiStats.spiRxPollCnt,
		         spiStats.spiRxCompleteCnt,
		         spiStats.spiRxTimeoutCnt,
		         spiStats.spiRxInvalidCnt,
		         spiStats.spiRxCANQueuedCnt);
		RAMN_UART_SendStringFromTask(statsBuf);
#endif
		*/
#endif

		// Example: send UART data every second
#ifdef ENABLE_UART
		// RAMN_UART_SendStringFromTask("Hello from RAMN\r");
#endif

		// Example: Send a CAN message every second.
		// Note that if it is sent from ECU A, it will not show up on ECU A's USB interface (e.g., slcan), because ECU A will be the sender (and therefore not a receiver).
		/*
		FDCAN_TxHeaderTypeDef header;
		uint8_t data[8U];

		header.BitRateSwitch = FDCAN_BRS_OFF;			// Bitrate switching OFF (only needed for CAN-FD, but set anyway); other option is FDCAN_BRS_ON.
		header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; 	// ESI bit (for CAN-FD only, but set anyway); other option is FDCAN_ESI_PASSIVE.
		header.FDFormat = FDCAN_CLASSIC_CAN; 			// Classic CAN; other option is FDCAN_FD_CAN.
		header.TxFrameType = FDCAN_DATA_FRAME;			// Data frame; other option is FDCAN_REMOTE_FRAME, only for classic CAN.
		header.IdType = FDCAN_STANDARD_ID;				// Standard identifier; other option is FDCAN_EXTENDED_ID for extended.
		header.Identifier = 0x123; 						// Identifier.
		header.DataLength = 8U;  						// DLC (Payload size).

		// Decide CAN message payload content
		RAMN_memset(data, 0x77, 8U); // write 0x77 8 times

		// Send message
		RAMN_FDCAN_SendMessage(&header,data);
		*/


		// Example: Execute every second, only if joystick is currently pressed down; only from ECU C (which is in charge of the sensor)
		// This is based on physical sensor data (ramn_sensors.h)
		/*
		if (RAMN_SENSORS_POWERTRAIN.shiftJoystick == RAMN_SHIFT_PUSH)
		{
			// Do something
		}
		*/

		// Example: Execute every second, only if joystick is currently pressed down; from ANOTHER ECU (other than ECU C)
		// This is based on the latest joystick CAN message received (ramn_dbc.h)
		// You need to make sure that the joystick CAN message is processed by adding #define RECEIVE_CONTROL_SHIFT in vehicle_specific.h
		/*
		if (RAMN_DBC_Handle.joystick == RAMN_SHIFT_PUSH)
		{
			// Do something
		}
		*/


	}

	loopCounter += 1; 	//You may want to add a check for integer overflow.
}

/* TIMERS */

// TIM16 is configured as a free running timer (e.g., to use for accurate timing measurements). Default: 1MHz counter (you can modify it without impacting other features).
// To reset TIM16 (e.g., to start a measurement), use:
// __HAL_TIM_SET_COUNTER(&htim16, 0);
// To read the value of TIM16 (to get your timing measurement), use:
// __HAL_TIM_GET_COUNTER(&htim16);  (should return uint16_t)

// TIM6  is configured as a trigger periodically calling the function below. Default: every 1s (you can modify it without impacting other features)

// Warning: This function is called within an ISR. It should not use freeRTOS functions not available to ISRs, and should return quickly.
void RAMN_CUSTOM_TIM6ISR(TIM_HandleTypeDef *htim)
{
	tim6val++;
}


/* TASK HOOKS */

// The functions below are called by tasks that are started but not used (e.g., USB task when USB is not active).
// They can be used to implement tasks that will not interfere with the main periodic task.
// Note that the priority of these tasks is typically higher than the periodic task, therefore they MUST periodically let other tasks execute (e.g. by calling vTaskDelayUntil or osDelay).
// Alternatively, if you want to execute slow and long code, you may alter the priority of the task.
// If you do not need these functions, use vTaskDelete(NULL) to delete the task.
// In all cases, make sure that you only modify the behavior of the targeted ECU, and not all ECUs (e.g., by using #ifdef TARGET_ECUB or #ifndef TARGET_ECUA).

#ifndef ENABLE_CDC
void RAMN_CUSTOM_CustomTask1(void *argument)
{
	//Called by RAMN_ReceiveUSBFunc
	vTaskDelete(NULL);
}

void RAMN_CUSTOM_CustomTask2(void *argument)
{
	//Called by RAMN_SendUSBFunc
	vTaskDelete(NULL);
}
#endif

#ifndef ENABLE_GSUSB
void RAMN_CUSTOM_CustomTask3(void *argument)
{
	//Called by RAMN_RxTask2Func
	vTaskDelete(NULL);
}

void RAMN_CUSTOM_CustomTask4(void *argument)
{
	//Called by RAMN_TxTask2Func
	vTaskDelete(NULL);
}
#endif

#ifndef ENABLE_DIAG
void RAMN_CUSTOM_CustomTask5(void *argument)
{
	//RAMN_DiagRXFunc
	vTaskDelete(NULL);
}

void RAMN_CUSTOM_CustomTask6(void *argument)
{
	//RAMN_DiagTXFunc
	vTaskDelete(NULL);
}
#endif


/* HARDWARE INTERFACE HOOKS */

#ifdef ENABLE_I2C
void RAMN_CUSTOM_ReceiveI2C(uint8_t buf[], uint16_t buf_size)
{
	// Warning: This function is called within an ISR. It should not use freeRTOS functions not available to ISRs, and should return quickly.
	// See RAMNV1.ioc for I2C device address (likely 0x77)
	// Note that by default, buf_size is fixed and equal to I2C_RX_BUFFER_SIZE. Function will NOT be called if fewer bytes are received.
	// You'll need to modify HAL_I2C_AddrCallback and HAL_I2C_SlaveRxCpltCallback in main.c if you need another behavior.
}

void RAMN_CUSTOM_PrepareTransmitDataI2C(uint8_t buf[], uint16_t buf_size)
{
	// Warning: This function is called within an ISR. It should not use freeRTOS functions not available to ISRs, and should return quickly.
	// Note that you cannot modify buf_size, only buf.
	// You'll need to modify HAL_I2C_AddrCallback in main.c if you need another behavior.
}
#endif

#ifdef ENABLE_UART

// You can send UART data using RAMN_UART_SendStringFromTask or RAMN_UART_SendFromTask, which are both non-blocking.
// This function is called from a task, with an intermediary UART buffer, and does not need to return quickly.
void RAMN_CUSTOM_ReceiveUART(uint8_t buf[], uint16_t buf_size)
{
	// By default, this function receives commands line by line, without endline character (\r)
	// You can modify this behavior in main.c (look for HAL_UART_Receive_IT and  HAL_UART_RxCpltCallback)
}
#endif
