/*
 * ramn_telematics.h
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

// This module provides telematics functionality for CAN-to-SPI bridge
#ifndef INC_RAMN_TELEMATICS_H_
#define INC_RAMN_TELEMATICS_H_

#include "ramn_msg_types.h"

#include "main.h"

// Public API Functions
void RAMN_TELEMATICS_Init(uint32_t tick);
void RAMN_TELEMATICS_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick);
void RAMN_TELEMATICS_Update(uint32_t tick);

// Returns True while a keyframe or delta stream is in progress (reduces poll interval to 1 ms).
RAMN_Bool_t RAMN_TELEMATICS_IsStreamActive(void);

// Called when CAN 0x303 (IMG_ACK) is received from ECU A.
void RAMN_TELEMATICS_ProcessImageACK(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick);

// SPI DMA Callbacks (called from ramn_spi.c)
void RAMN_TELEMATICS_SPI_TxCpltCallback(void);
void RAMN_TELEMATICS_SPI_TxRxCpltCallback(void);

// Flag indicating custom expansion board SPI transmission is in progress
// Used by ramn_spi.c to distinguish between screen and custom board DMA completions
extern volatile RAMN_Bool_t spiTransmitBusy;

#endif /* INC_RAMN_TELEMATICS_H_ */
