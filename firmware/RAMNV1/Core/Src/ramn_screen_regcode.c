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

#ifdef ENABLE_REGCODE_SECOC
#include "ramn_secoc.h"
#include "ramn_secoc_link.h"
#endif

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

#ifdef ENABLE_REGCODE_SECOC
// ============================================================================
// SecOC -- RECEIVER SIDE
//
// What this enforces is that the six digits on the panel came from ECU D, and
// from ECU D just now. Neither half is optional: without the first, anyone on
// the bus can display a code of their choosing; without the second, anyone who
// recorded a genuine one can show it again tomorrow, and a one-time code that
// can be shown twice is not one.
//
// The key and the session belong to ramn_secoc_link.c, dispatched from main.c.
// This module owns only the freshness domain for THIS message -- see the note
// beside ECU D's regFv in ramn_telematics.c for why it is not the image
// stream's.
// ============================================================================
volatile uint16_t RAMN_SCREENREGCODE_NoSessionDrops = 0U;
volatile uint16_t RAMN_SCREENREGCODE_AuthFailures   = 0U;

// Deliberately NOT reset by SCREENREGCODE_Deinit, which purges everything else
// this module holds. The counter is a high-water mark, and forgetting it is
// exactly what a replay needs: leave the screen, and the code that was just
// shown becomes acceptable again.
static RAMN_SecOC_Freshness_t regFv;
static uint32_t               regSessionGen = 0U;
#endif

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
	// Note: CAN ID, frame format and length are validated in the screen manager
	// before this is called. Authenticity is NOT -- that is this function's job,
	// and it is done here rather than there because the freshness domain and the
	// session generation that goes with it belong to this module.

#ifdef ENABLE_REGCODE_SECOC
	// FAIL CLOSED. No session, no code on the panel. Falling back to displaying
	// an unverified code would mean an attacker only has to keep the handshake
	// from completing to get whatever six digits they like in front of the
	// driver -- and denying the handshake is the easy half.
	if (RAMN_SecOC_LINK_Ready() == 0U)
	{
		if (RAMN_SCREENREGCODE_NoSessionDrops < 0xFFFFU) RAMN_SCREENREGCODE_NoSessionDrops++;
		return;
	}

	// A rekey restarts the counters at zero, so a high-water mark from the
	// previous session would reject every message of this one. Safe to forget
	// precisely because the key changed with it.
	{
		uint32_t gen = RAMN_SecOC_LINK_Generation();
		if (gen != regSessionGen)
		{
			regSessionGen = gen;
			RAMN_SecOC_FreshnessInit(&regFv);
		}
	}

	// Verification runs BEFORE the screen-state gates below, not after, and the
	// order is the point. The freshness counter has to follow every authentic
	// message, not only the ones acted on: a genuine code that arrives while
	// the screen is still busy is dropped either way, but if its freshness were
	// never recorded, that same frame stays acceptable and can be replayed the
	// moment the cooldown expires.
	//
	// Reconstruct, verify UNDER the reconstructed value, and only then advance.
	// Advancing first would let anyone walk the counter forward with garbage
	// and lock ECU D out for the rest of the session.
	uint32_t trunc = ((uint32_t)data[REGCODE_SECOC_FV_OFFSET] << 8) |
	                  (uint32_t)data[REGCODE_SECOC_FV_OFFSET + 1U];
	uint32_t full;

	// Stale or beyond the window: cheap to reject, and rejecting it here is
	// what keeps a flood of replayed frames from costing a BLAKE2s each on the
	// CAN RX task.
	if (RAMN_SecOC_RxFreshness(&regFv, trunc, REGCODE_SECOC_FV_TRUNC_BITS, &full) == 0U)
	{
		if (RAMN_SCREENREGCODE_AuthFailures < 0xFFFFU) RAMN_SCREENREGCODE_AuthFailures++;
		return;
	}

	{
		RAMN_SecOC_Ctx_t ctx;
		ctx.dataId = (uint16_t)REGCODE_CAN_ID;
		ctx.macLen = REGCODE_SECOC_MAC_BYTES;
		// The SESSION key, never the provisioned root. The root is spent once,
		// on the handshake in ramn_secoc_link.c.
		ctx.key    = RAMN_SecOC_LINK_Key();
		ctx.fv     = &regFv;

		uint8_t authLen = (uint8_t)(REGCODE_CAN_FRAME_BYTES - REGCODE_SECOC_MAC_BYTES);
		if (RAMN_SecOC_CheckMac(&ctx, full, data, authLen, &data[authLen]) == 0U)
		{
			if (RAMN_SCREENREGCODE_AuthFailures < 0xFFFFU) RAMN_SCREENREGCODE_AuthFailures++;
			return;
		}
	}

	RAMN_SecOC_RxAccept(&regFv, full);
#endif /* ENABLE_REGCODE_SECOC */

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

	// Bytes past the code carry the SecOC freshness value and authenticator
	// (see ramn_config.h). With ENABLE_REGCODE_SECOC off they are unused, and
	// the frame is the eight-byte one that shipped before authentication.

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
