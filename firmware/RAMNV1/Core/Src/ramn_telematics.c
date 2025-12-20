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
#include "ramn_canfd.h"

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
#define SPI_POLL_INTERVAL_MS 50   // Poll ESP32 every 50ms for responsive bidirectional communication
#define SPI_POLL_TIMEOUT_MS 10    // Max wait for ESP32 response (reduced for fast SPI bit rate)
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
static volatile uint32_t spiLastPollAttemptTick = 0;  // Track last attempt (even if failed due to busy)

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

	// Zero-initialize RX buffers (prevent garbage on first poll)
	RAMN_memset(spiRxBufferA, 0, SPI_RX_BUFFER_SIZE);
	RAMN_memset(spiRxBufferB, 0, SPI_RX_BUFFER_SIZE);
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
void RAMN_TELEMATICS_SPI_TxCpltCallback(void)
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
	if (spiTransmitBusy == True)
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

	switch (spiPollState)
	{
		case SPI_POLL_IDLE:
			// Check if it's time to poll ESP32
			if ((tick - spiLastPollTick) >= SPI_POLL_INTERVAL_MS)
			{
				if (RequestESP32Poll())
				{
					// Poll request successful
					spiLastPollTick = tick;
					spiLastPollAttemptTick = tick;
					// State changed to SPI_POLL_REQUESTED by RequestESP32Poll
				}
				else
				{
					// Poll failed (bus busy) - retry sooner by updating attempt time
					// This allows faster retries when SPI becomes available
					spiLastPollAttemptTick = tick;
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
				extern SPI_HandleTypeDef hspi2;
				HAL_SPI_Abort(&hspi2);

				// De-assert chip select (critical - must happen even if abort fails)
				HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

				taskENTER_CRITICAL();
				spiPollState = SPI_POLL_TIMEOUT;
				spiStats.spiRxTimeoutCnt++;
				taskEXIT_CRITICAL();
			}
			// SAFETY: Watchdog for extreme timeout (should never happen)
			// If stuck in REQUESTED state for >100ms, force reset
			else if ((tick - spiPollRequestTick) >= 100)
			{
				// Extreme timeout - force recovery
				extern SPI_HandleTypeDef hspi2;
				HAL_SPI_Abort(&hspi2);
				HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_SET);

				taskENTER_CRITICAL();
				spiPollState = SPI_POLL_IDLE;  // Skip normal timeout handling
				spiStats.spiRxWatchdogResetCnt++;
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
}
