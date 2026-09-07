#pragma once
#include "main.h"
/* The real type codes, not a copy of them. */
#include "ramn_msg_types.h"
void RAMN_TELEMATICS_Init(uint32_t tick);
void RAMN_TELEMATICS_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick);
void RAMN_TELEMATICS_Update(uint32_t tick);
RAMN_Bool_t RAMN_TELEMATICS_IsStreamActive(void);
void RAMN_TELEMATICS_ProcessImageACK(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick);
void RAMN_TELEMATICS_SPI_TxCpltCallback(void);
void RAMN_TELEMATICS_SPI_TxRxCpltCallback(void);
extern volatile RAMN_Bool_t spiTransmitBusy;
