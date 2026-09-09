/*
 * ramn_secoc_link.h
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

// The SecOC security association between two ECUs, and the CAN plumbing that
// negotiates it.
//
// WHY THIS IS ITS OWN MODULE
//
// A session is an ECU-level fact, not a property of whatever happens to use
// it. It outlives any one screen, any one stream, and any one protocol; it is
// established once and then answers "what key, and what freshness domain?" for
// everything that asks.
//
// It first lived inside ramn_screen_image.c, which was wrong in three ways
// that showed up immediately: the handshake state had to survive
// SCREENIMAGE_Init even though every other thing in that module is reset by
// it; RAMN_SCREENMANAGER_ProcessRxCANMessage had to route four CAN IDs that
// have nothing to do with screens; and the identical frame-building and
// nonce-drawing code existed a second time in ramn_telematics.c for ECU D.
// Hooked from main.c as a peer of RAMN_DIAG_ / RAMN_TELEMATICS_ /
// RAMN_CUSTOM_ProcessRxCANMessage, all three go away.
//
// WHAT STAYS WITH THE CALLER
//
// Per-message verification does NOT belong here, because it needs the calling
// protocol's own grouping rules -- an image chunk is verified under the
// freshness its keyframe opened, which only ramn_screen_image.c knows. This
// module owns the key and the freshness domain; the caller decides what to
// check and when. See RAMN_SecOC_LINK_Key / _Freshness.
//
// ROLES
//
// Exactly one role is compiled per ECU, from ramn_config.h:
//
//   SECOC_LINK_ROLE_VERIFIER  the side that receives protected messages and
//                             answers challenges (ECU A)
//   SECOC_LINK_ROLE_SENDER    the side that sends them and asks for sessions
//                             (ECU D)
//
// Compiling only one matters: a verifier that also answered SESSION_CHALLENGE
// would hand anyone on the bus a transcript-MAC oracle for the asking.

#ifndef INC_RAMN_SECOC_LINK_H_
#define INC_RAMN_SECOC_LINK_H_

#include "main.h"
#include "ramn_secoc.h"
#include "ramn_secoc_session.h"

#ifdef ENABLE_IMAGE_SECOC

// Clears any session. Call once at startup, after the key store is loaded.
void RAMN_SecOC_LINK_Init(void);

// Feeds one received CAN frame to the handshake. Ignores anything that is not
// a session message, so it is safe to call for every frame. Hooked in main.c
// beside the other ProcessRxCANMessage handlers.
void RAMN_SecOC_LINK_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader,
                                         const uint8_t* data, uint32_t tick);

// 1 when a session is established. Everything protected must refuse while this
// is 0 -- that is what failing closed means.
uint8_t RAMN_SecOC_LINK_Ready(void);

// The session key. Never the provisioned root, which does no per-message work.
// Points at zeroes while no session is up, which is why callers must gate on
// RAMN_SecOC_LINK_Ready rather than on this being non-NULL.
const uint8_t* RAMN_SecOC_LINK_Key(void);

// The freshness domain that belongs to the current session. Reset to zero when
// a session is adopted, which is safe precisely because the key changed with
// it.
RAMN_SecOC_Freshness_t* RAMN_SecOC_LINK_Freshness(void);

// Increments every time a new session is adopted. Callers that cache anything
// derived from a session -- a per-frame freshness value, say -- compare this
// against what they last saw and start over when it moves. Without it, a
// stale value from the previous session is checked against a counter that
// restarted at zero, and every message fails for the wrong reason.
uint32_t RAMN_SecOC_LINK_Generation(void);

// How many session messages were refused because they did not verify. Worth
// surfacing: an attacker probing the handshake and a quiet bus look identical
// otherwise.
uint16_t RAMN_SecOC_LINK_FailedCount(void);

#ifdef SECOC_LINK_ROLE_SENDER
// Asks for a session if there is not one, rate limited. Returns 1 when a
// session is already up and the caller may transmit, 0 when it must not.
uint8_t RAMN_SecOC_LINK_EnsureSession(uint32_t tick);

// Drops the session so the next EnsureSession re-handshakes. For the case
// where the far side has stopped answering: it has most likely rebooted and no
// longer holds the key, and it cannot say so, because any message it sent
// would itself need a session.
void RAMN_SecOC_LINK_Drop(void);
#endif

#endif /* ENABLE_IMAGE_SECOC */

#endif /* INC_RAMN_SECOC_LINK_H_ */
