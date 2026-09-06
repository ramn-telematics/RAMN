/*
 * ramn_screen_regcode.c
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

#include "ramn_screen_regcode.h"

#ifdef ENABLE_SCREEN

// Flag to indicate screen should be displayed
volatile RAMN_Bool_t RAMN_SCREENREGCODE_DisplayRequested = False;

// Timeout duration in milliseconds (10 seconds)
#define REGCODE_TIMEOUT_MS 10000U
// Cooldown period after timeout before accepting new codes (2 seconds)
#define REGCODE_COOLDOWN_MS 2000U

// Variables to store registration code and timing
static uint32_t registrationCode = 0;
static uint32_t screenActivatedTick = 0;
static RAMN_Bool_t screenActive = False;
static uint32_t lastTimeoutTick = 0;  // Track when last timeout occurred for cooldown

// Private function to format 6-digit code with leading zeros
static void formatRegCode(uint32_t code, char* buffer)
{
	// Ensure code is within 6-digit range (0-999999)
	code = code % 1000000;

	// Format as 6 digits with leading zeros
	buffer[0] = '0' + ((code / 100000) % 10);
	buffer[1] = '0' + ((code / 10000) % 10);
	buffer[2] = '0' + ((code / 1000) % 10);
	buffer[3] = '0' + ((code / 100) % 10);
	buffer[4] = '0' + ((code / 10) % 10);
	buffer[5] = '0' + (code % 10);
	buffer[6] = '\0';
}

// Draw large registration code using 2x scaled characters
static void drawLargeRegCode(uint16_t x, uint16_t y, uint16_t fgColor, uint16_t bgColor, const char* code)
{
	uint16_t spacing = 36;  // Space between 2x scaled characters (32px char width + 4px gap)
	for (int i = 0; i < 6 && code[i] != '\0'; i++)
	{
		RAMN_SPI_DrawLargeChar(x + (i * spacing), y, fgColor, bgColor, code[i], 2);
	}
}

// Refresh version for updates without redrawing background
static void refreshLargeRegCode(uint16_t x, uint16_t y, uint16_t fgColor, uint16_t bgColor, const char* code)
{
	uint16_t spacing = 36;  // Match the spacing from drawLargeRegCode
	for (int i = 0; i < 6 && code[i] != '\0'; i++)
	{
		RAMN_SPI_DrawLargeChar(x + (i * spacing), y, fgColor, bgColor, code[i], 2);
	}
}

static void SCREENREGCODE_Init()
{
	RAMN_SCREENUTILS_DrawBase();

	// Draw title at top
	RAMN_SPI_DrawString(25, 5, RAMN_SCREENUTILS_COLORTHEME.LIGHT, RAMN_SCREENUTILS_COLORTHEME.BACKGROUND, "Registration Code");

	// Draw initial placeholder or actual code
	char codeBuffer[7];
	formatRegCode(registrationCode, codeBuffer);

	// Draw large code in center with better positioning
	// 6 chars * 36px spacing = 216px total width, centered at 240px wide screen: (240-216)/2 = 12px offset
	drawLargeRegCode(20, 80, RAMN_SCREENUTILS_COLORTHEME.WHITE, RAMN_SCREENUTILS_COLORTHEME.BACKGROUND, codeBuffer);

	// Draw timeout indicator at bottom
	RAMN_SPI_DrawString(30, 150, RAMN_SCREENUTILS_COLORTHEME.MEDIUM, RAMN_SCREENUTILS_COLORTHEME.BACKGROUND, "Auto-close in 10s");
}

static void SCREENREGCODE_Update(uint32_t tick)
{
	// Check for timeout
	if (screenActive && (tick - screenActivatedTick > REGCODE_TIMEOUT_MS))
	{
		screenActive = False;
		RAMN_SCREENREGCODE_DisplayRequested = False;
		registrationCode = 0;  // Purge the code from memory
		lastTimeoutTick = tick;  // Record when timeout occurred for cooldown
		// Clearing the DisplayRequested flag will cause screen manager to no longer
		// force this screen to be active, allowing navigation away
	}

	// Update code display if needed (only while screen is active)
	if (screenActive && (RAMN_SCREENUTILS_LoopCounter % 5U == 0U))
	{
		//char codeBuffer[7];
		//formatRegCode(registrationCode, codeBuffer);

		// Refresh the code display with large font
		//refreshLargeRegCode(12, 60, RAMN_SCREENUTILS_COLORTHEME.WHITE, RAMN_SCREENUTILS_COLORTHEME.BACKGROUND, codeBuffer);

		// Update remaining time
		uint32_t elapsed = tick - screenActivatedTick;
		uint32_t remaining = (REGCODE_TIMEOUT_MS - elapsed) / 1000U;

		if (remaining <= 10)
		{
			char timeBuffer[20];
			snprintf(timeBuffer, sizeof(timeBuffer), "Auto-close in %lus ", remaining);
			RAMN_SPI_RefreshString(30, 150, RAMN_SCREENUTILS_COLORTHEME.MEDIUM, RAMN_SCREENUTILS_COLORTHEME.BACKGROUND, timeBuffer);
		}

		RAMN_SCREENUTILS_DrawSubconsoleUpdate();
	}
}

static void SCREENREGCODE_Deinit()
{
	// Reset state and purge code when leaving screen
	screenActive = False;
	RAMN_SCREENREGCODE_DisplayRequested = False;
	registrationCode = 0;  // Purge the code from memory
}

static RAMN_Bool_t SCREENREGCODE_UpdateInput(JoystickEventType event)
{
	// Block all joystick navigation while code is active (not expired)
	if (screenActive)
	{
		return False;  // Don't allow screen manager to process input
	}
	return True;  // Allow navigation after timeout
}

static void SCREENREGCODE_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader, const uint8_t* data, uint32_t tick)
{
	// Note: Validation is done in screen manager before this is called
	// We only get here if CAN ID matches and frame is valid

	// Ignore messages if screen is already active (prevents timer resets)
	if (screenActive)
	{
		return;
	}

	// Ignore messages during cooldown period after timeout (prevents immediate re-trigger)
	if (lastTimeoutTick != 0 && (tick - lastTimeoutTick < REGCODE_COOLDOWN_MS))
	{
		return;
	}

	// Extract 32-bit registration code from bytes 0-3 (little-endian format, LSB first)
	registrationCode = (uint32_t)data[0] |
	                   ((uint32_t)data[1] << 8) |
	                   ((uint32_t)data[2] << 16) |
	                   ((uint32_t)data[3] << 24);

	// Bytes 4-7 are reserved for future signing algorithm
	// uint32_t signature = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
	//                      ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
	// TODO: Add signature validation here when signing algorithm is implemented

	// Activate screen and set timeout timer
	screenActive = True;
	screenActivatedTick = tick;
	RAMN_SCREENREGCODE_DisplayRequested = True;
	lastTimeoutTick = 0;  // Clear cooldown timer when new code is accepted
}

RAMNScreen_t ScreenRegCode = {
	.Init = SCREENREGCODE_Init,
	.Update = SCREENREGCODE_Update,
	.Deinit = SCREENREGCODE_Deinit,
	.UpdateInput = SCREENREGCODE_UpdateInput,
	.ProcessRxCANMessage = SCREENREGCODE_ProcessRxCANMessage
};

#endif
