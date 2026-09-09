/*
 * ramn_secoc_keys.h
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

// Where a SecOC key comes from. Split out of ramn_secoc.c on purpose: that
// module is portable and knows nothing about this board, and this one is all
// board.
//
// The key is held in the emulated EEPROM (AN4894), four 32-bit variables at
// RAMN_SECOC_EEPROM_INDEX. If none has been written, the compile-time
// RAMN_SECOC_DEFAULT_KEY is used instead, so a freshly flashed pair of ECUs
// streams images on first boot with no provisioning step at all.
//
// THE DEFAULT KEY IS NOT A SECRET. It is a constant in a public repository and
// is identical on every RAMN ever built. It exists so the feature can be
// brought up and tested; it authenticates nothing against anyone who can read
// this file. RAMN_SecOC_KEYS_IsProvisioned tells you which of the two you are
// running on, and the image ACK reports it on the bus.
//
// EEPROM vs OTP. Emulated EEPROM is the right place to prove this out: it is
// rewritable, so a botched provisioning run is fixable, and iterating on the
// key format costs nothing. It has two properties to be aware of before
// trusting it:
//
//   - It is erased with the firmware. The emulation pages live in flash at
//     START_PAGE_ADDRESS (0x0803E000), and scripts/STbootloader passes -e
//     (mass erase) on every reprogram, so reflashing an ECU un-provisions it
//     and drops it back to the default key.
//   - It is writable at runtime. Anything that can reach a flash write can
//     replace the key with one it knows, which forges images rather than
//     merely reading them.
//
// The OTP area at 0x0BFA0000 has neither property -- it survives reflashing
// and cannot be rewritten at all -- at the cost of being burn-once. Moving
// there is a change to this file only; nothing above RAMN_SecOC_KEYS_GetImage
// knows the difference.

#ifndef INC_RAMN_SECOC_KEYS_H_
#define INC_RAMN_SECOC_KEYS_H_

#include <stdint.h>
#include "ramn_config.h"
#include "ramn_secoc.h"

// First of four consecutive 32-bit EEPROM variables holding the image key.
// Sits in the 0x0100-0xEFFF range the EEPROM layer leaves to applications.
#define RAMN_SECOC_EEPROM_INDEX   0x0200U

// NOTE ON WHAT THIS KEY IS FOR
//
// It is the ROOT key. It authenticates the session handshake once
// (ramn_secoc_session.h) and does no per-frame work -- image messages are
// authenticated under the session key derived from it. So the exposure of this
// value is a few frames per session rather than hundreds per keyframe, and
// swapping it for an OTP-held key later changes nothing above
// RAMN_SecOC_KEYS_GetImageKey.

// Loads the image stream key. Call once at startup, before any image traffic.
void RAMN_SecOC_KEYS_Init(void);

// The active image stream key, RAMN_SECOC_KEY_BYTES long. Never NULL: falls
// back to the default key when the EEPROM holds nothing.
const uint8_t* RAMN_SecOC_KEYS_GetImageKey(void);

// 1 when the active key came from the EEPROM, 0 when it is the built-in
// default. Worth surfacing -- a pair of ECUs silently running the public
// default key looks exactly like a pair running a real one.
uint8_t RAMN_SecOC_KEYS_IsProvisioned(void);

// Writes a new image stream key to the EEPROM and adopts it immediately.
// Returns 1 on success. Takes RAMN_SECOC_KEY_BYTES bytes.
uint8_t RAMN_SecOC_KEYS_SetImageKey(const uint8_t* key);

#endif /* INC_RAMN_SECOC_KEYS_H_ */
