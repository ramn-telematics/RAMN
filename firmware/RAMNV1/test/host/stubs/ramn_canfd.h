#pragma once
#include "main.h"
RAMN_Result_t RAMN_FDCAN_SendMessage(const FDCAN_TxHeaderTypeDef* header, const uint8_t* data);
extern StreamBufferHandle_t CANTxDataStreamBufferHandle;

/* Mirrors the fields of RAMN_FDCAN_Status the modules under test read. Kept as
   a real object, not a macro, so a test can set CANRxOverrunCnt and assert that
   it reaches the 0x303 ACK. */
typedef struct {
    volatile uint32_t CANRXCnt;
    volatile uint32_t CANRxOverrunCnt;
} RAMN_FDCAN_Status_t;
extern RAMN_FDCAN_Status_t RAMN_FDCAN_Status;
