/*
 * ramn_secoc_keys.c
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

#include "ramn_secoc_keys.h"

#if defined(ENABLE_EEPROM_EMULATION)
#include "ramn_eeprom.h"
#endif

// See the header: this is a published constant, not a secret. It only has to
// be the same on ECU A and ECU D.
static const uint8_t secocDefaultKey[RAMN_SECOC_KEY_BYTES] =
{
	0x52U, 0x41U, 0x4DU, 0x4EU, 0x53U, 0x45U, 0x43U, 0x4FU,
	0x43U, 0x44U, 0x45U, 0x46U, 0x41U, 0x55U, 0x4CU, 0x54U
};

static uint8_t secocImageKey[RAMN_SECOC_KEY_BYTES];
static uint8_t secocProvisioned = 0U;

static void LoadDefault(void)
{
	for (uint8_t i = 0U; i < RAMN_SECOC_KEY_BYTES; i++) secocImageKey[i] = secocDefaultKey[i];
	secocProvisioned = 0U;
}

void RAMN_SecOC_KEYS_Init(void)
{
#if defined(ENABLE_EEPROM_EMULATION)
	uint32_t words[RAMN_SECOC_KEY_BYTES / 4U];

	for (uint8_t w = 0U; w < (RAMN_SECOC_KEY_BYTES / 4U); w++)
	{
		// Any word missing means the key was never fully written. A partial
		// key is worse than no key: it would authenticate nothing while
		// looking provisioned, so treat the whole thing as absent.
		if (RAMN_EEPROM_Read32((uint16_t)(RAMN_SECOC_EEPROM_INDEX + w), &words[w]) != EE_OK)
		{
			LoadDefault();
			return;
		}
	}

	// An all-zero key is what a half-erased or zero-filled EEPROM looks like.
	// Refuse it rather than authenticating under a key an attacker can guess
	// without reading anything.
	uint32_t any = 0U;
	for (uint8_t w = 0U; w < (RAMN_SECOC_KEY_BYTES / 4U); w++) any |= words[w];
	if (any == 0U)
	{
		LoadDefault();
		return;
	}

	for (uint8_t w = 0U; w < (RAMN_SECOC_KEY_BYTES / 4U); w++)
	{
		secocImageKey[(w * 4U) + 0U] = (uint8_t)((words[w] >> 24) & 0xFFU);
		secocImageKey[(w * 4U) + 1U] = (uint8_t)((words[w] >> 16) & 0xFFU);
		secocImageKey[(w * 4U) + 2U] = (uint8_t)((words[w] >>  8) & 0xFFU);
		secocImageKey[(w * 4U) + 3U] = (uint8_t)( words[w]        & 0xFFU);
	}
	secocProvisioned = 1U;
#else
	LoadDefault();
#endif
}

const uint8_t* RAMN_SecOC_KEYS_GetImageKey(void)
{
	return secocImageKey;
}

uint8_t RAMN_SecOC_KEYS_IsProvisioned(void)
{
	return secocProvisioned;
}

uint8_t RAMN_SecOC_KEYS_SetImageKey(const uint8_t* key)
{
	if (key == NULL) return 0U;

#if defined(ENABLE_EEPROM_EMULATION)
	for (uint8_t w = 0U; w < (RAMN_SECOC_KEY_BYTES / 4U); w++)
	{
		uint32_t val = ((uint32_t)key[(w * 4U) + 0U] << 24) |
		               ((uint32_t)key[(w * 4U) + 1U] << 16) |
		               ((uint32_t)key[(w * 4U) + 2U] <<  8) |
		               ((uint32_t)key[(w * 4U) + 3U]);
		if (RAMN_EEPROM_Write32((uint16_t)(RAMN_SECOC_EEPROM_INDEX + w), val) != EE_OK) return 0U;
	}

	for (uint8_t i = 0U; i < RAMN_SECOC_KEY_BYTES; i++) secocImageKey[i] = key[i];
	secocProvisioned = 1U;
	return 1U;
#else
	return 0U;
#endif
}
