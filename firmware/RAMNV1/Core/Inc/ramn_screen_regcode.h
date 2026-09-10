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

// REGCODE_CAN_ID and the wire layout live in ramn_config.h: ECU D builds this
// frame and never compiles a screen module.

// Flag set when a new registration code is received and screen should be displayed
extern volatile RAMN_Bool_t RAMN_SCREENREGCODE_DisplayRequested;

#ifdef ENABLE_REGCODE_SECOC
// ECU A has no UART and no report channel of its own for this message -- the
// image stream borrows its 0x303 ACK, and inventing a second ACK ID for a
// message sent a few times a session would put more traffic on the bus than it
// is worth. So these two are what a debugger, or a UDS read, can look at.
//
// They answer different questions and must not be added together. The first
// says the link never came up; the second says someone is putting frames on
// the bus that do not verify. A quiet bus and one that is being injected into
// look identical without them.
extern volatile uint16_t RAMN_SCREENREGCODE_NoSessionDrops;
extern volatile uint16_t RAMN_SCREENREGCODE_AuthFailures;
#endif

extern RAMNScreen_t ScreenRegCode;

#endif

#endif /* INC_RAMN_SCREEN_REGCODE_H_ */
