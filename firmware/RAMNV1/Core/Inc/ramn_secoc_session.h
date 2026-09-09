/*
 * ramn_secoc_session.h
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

// Authenticated session establishment for SecOC.
//
// The provisioned key (ramn_secoc_keys.h) is a long-lived secret shared by the
// two ECUs on one board. This module spends it ONCE per session, to prove
// identity and to agree a fresh key; every protected message afterwards is
// authenticated under the derived session key, never under the root.
//
// WHY, BEYOND HYGIENE
//
// A session key closes the replay hole that a RAM-resident freshness counter
// leaves open. A receiver that reboots forgets the highest counter it has
// seen, so a frame recorded before the reboot would verify after it. Under a
// per-session key it does not: it was authenticated with a key that no longer
// exists. This is a better answer than persisting the counter -- no flash
// wear, no write on the hot path -- and it also bounds how much traffic is
// ever produced under the root key.
//
// THE EXCHANGE
//
//   D -> A   0x307  SESSION_REQ        (ECU D wants to stream)
//   A -> D   0x308  SESSION_CHALLENGE  nonce_A
//   D -> A   0x309  SESSION_RESPONSE   nonce_D | MAC_root(nonce_A | nonce_D)
//   A -> D   0x30A  SESSION_CONFIRM             MAC_root(nonce_D | nonce_A)
//
//   K_session = BLAKE2s(key = K_root, "RAMN-SecOC-session-v1" | nonce_A | nonce_D)
//
// BOTH SIDES CONTRIBUTE A NONCE, and that is load-bearing rather than
// symmetry for its own sake. If only ECU D chose one, an attacker who recorded
// an earlier handshake could replay it and walk ECU A back onto a previous
// session key -- which would revive every frame they recorded under it, the
// exact thing the session key exists to prevent. ECU A's own fresh nonce makes
// each derived key unreachable by replay.
//
// The exchange is MUTUAL. ECU A authenticates ECU D, which is the stated goal;
// ECU D also authenticates ECU A, so that an attacker posing as ECU A cannot
// leave ECU D streaming under a key the real ECU A does not hold. They could
// not read the images either way -- deriving K_session needs K_root -- but
// they could deny the link, and one extra frame per session closes it.
//
// REFLECTION
//
// Nothing in the two transcript MACs is symmetric: they are computed under
// different SecOC Data IDs (0x309 against 0x30A) AND over the nonces in
// opposite orders. A response cannot be replayed back as a confirm, in either
// direction. That falls out of reusing RAMN_SecOC_ComputeMac, which binds the
// Data ID into every authenticator it produces.
//
// A HANDSHAKE NEVER TEARS DOWN A LIVE SESSION
//
// SESSION_REQ and SESSION_CHALLENGE are unauthenticated by construction --
// they are what bootstraps authentication, so they cannot themselves be
// authenticated. Anyone on the bus can therefore send them. That is harmless
// only if a handshake in flight leaves the established session alone until a
// confirm actually verifies, so callers MUST negotiate into a scratch session
// and adopt it on success. RAMN_SecOC_SESSION_Adopt is the one place that
// swap happens.
//
// PORTABILITY
//
// Like ramn_secoc.h: C standard library and ramn_blake2s.h only. Rate limits,
// timeouts and the CAN plumbing belong to the caller, which is where the
// scheduling context is.

#ifndef INC_RAMN_SECOC_SESSION_H_
#define INC_RAMN_SECOC_SESSION_H_

#include <stdint.h>
#include <stddef.h>
#include "ramn_secoc.h"

// Per-side nonce. 8 bytes each, 128 bits of combined input to the KDF.
// The nonce's job is to make each derived key unique, not to be unguessable
// on its own -- an attacker who predicts both still cannot derive the key
// without K_root -- so 64 bits a side is ample for the few thousand sessions
// a board will ever run, and it keeps SESSION_RESPONSE inside one 16-byte
// CAN FD frame.
#define RAMN_SECOC_NONCE_BYTES        8U

// Authenticator on the two handshake messages. Longer than the 4 bytes an
// image frame carries: this runs once per session rather than hundreds of
// times per keyframe, and a forged handshake is a total compromise of the
// link rather than one bad chunk, so it is worth 2^64 instead of 2^32.
#define RAMN_SECOC_SESSION_MAC_BYTES  8U

typedef enum
{
	RAMN_SECOC_SESSION_NONE = 0,   // nothing agreed; protected traffic is refused
	RAMN_SECOC_SESSION_PENDING,    // a handshake is in flight
	RAMN_SECOC_SESSION_OK          // key agreed; protected traffic is verified
} RAMN_SecOC_SessionState_t;

typedef struct
{
	RAMN_SecOC_SessionState_t state;
	uint8_t                   nonceA[RAMN_SECOC_NONCE_BYTES];
	uint8_t                   nonceD[RAMN_SECOC_NONCE_BYTES];
	uint8_t                   key[RAMN_SECOC_KEY_BYTES];
	RAMN_SecOC_Freshness_t    fv;
} RAMN_SecOC_Session_t;

// Clears a session to NONE and wipes its key. Also the correct startup state.
void RAMN_SecOC_SESSION_Reset(RAMN_SecOC_Session_t* s);

// Derives the session key from the two nonces and stores it in s, leaving the
// state untouched -- see RAMN_SecOC_SESSION_Adopt for the commit step.
void RAMN_SecOC_SESSION_Derive(RAMN_SecOC_Session_t* s, const uint8_t* rootKey);

// Commits a negotiated session: marks it established and zeroes its freshness
// counters, so every session starts counting from the same place.
void RAMN_SecOC_SESSION_Adopt(RAMN_SecOC_Session_t* s);

// Transcript authenticator, bound to the CAN ID that carries it.
// Writes RAMN_SECOC_SESSION_MAC_BYTES to macOut over (first | second).
void RAMN_SecOC_SESSION_Mac(const uint8_t* rootKey, uint16_t dataId,
                            const uint8_t* first, const uint8_t* second,
                            uint8_t* macOut);

// 1 if macIn matches, 0 otherwise. Constant time.
uint8_t RAMN_SecOC_SESSION_CheckMac(const uint8_t* rootKey, uint16_t dataId,
                                    const uint8_t* first, const uint8_t* second,
                                    const uint8_t* macIn);

#endif /* INC_RAMN_SECOC_SESSION_H_ */
