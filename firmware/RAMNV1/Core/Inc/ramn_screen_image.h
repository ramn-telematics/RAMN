/*
 * ramn_screen_image.h
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

// ECU A image streaming screen.
// Handles keyframe (0x300-0x303) and delta tile (0x304-0x306) CAN-FD protocols.
// Images originate on the ESP32, are encoded as RGB565+RLE, and forwarded via ECU D.

#ifndef INC_RAMN_SCREEN_IMAGE_H_
#define INC_RAMN_SCREEN_IMAGE_H_

#include "main.h"

#ifdef ENABLE_SCREEN

#include "ramn_screen_utils.h"

// Set True when a keyframe or delta stream is active.
// Screen manager monitors this flag to force this screen to the foreground.
extern volatile RAMN_Bool_t RAMN_SCREENIMAGE_DisplayRequested;

// RAMNScreen_t instance — registered in ramn_screen_manager.
extern RAMNScreen_t ScreenImage;

#endif /* ENABLE_SCREEN */

#endif /* INC_RAMN_SCREEN_IMAGE_H_ */
