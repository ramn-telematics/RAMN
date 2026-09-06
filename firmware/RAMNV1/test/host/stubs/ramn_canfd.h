#pragma once
#include "main.h"
RAMN_Result_t RAMN_FDCAN_SendMessage(const FDCAN_TxHeaderTypeDef* header, const uint8_t* data);
extern StreamBufferHandle_t CANTxDataStreamBufferHandle;
