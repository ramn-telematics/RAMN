/*
 * ramn_screen_regcode.h
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

// This module implements a screen to display a 6-digit registration code received via CAN

#ifndef INC_RAMN_SCREEN_REGCODE_H_
#define INC_RAMN_SCREEN_REGCODE_H_

#include "main.h"

#ifdef ENABLE_SCREEN

#include "ramn_screen_utils.h"

// CAN ID for registration code message
#define REGCODE_CAN_ID 0x7A0

// Flag set when a new registration code is received and screen should be displayed
extern volatile RAMN_Bool_t RAMN_SCREENREGCODE_DisplayRequested;

extern RAMNScreen_t ScreenRegCode;

#endif

#endif /* INC_RAMN_SCREEN_REGCODE_H_ */
