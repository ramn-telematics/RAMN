/*
 * ramn_secoc.h
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

// SecOC -- Secure Onboard Communication, after AUTOSAR's SWS of the same name.
//
// This module answers one question for any CAN message on RAMN: "did the ECU
// that is supposed to send this actually send it, and is this the message it
// sent NOW rather than one recorded an hour ago?" It is deliberately generic:
// the image stream (0x300-0x306) is its first user, but nothing here knows
// about images. Give a message a Data ID, point it at a freshness domain, and
// it is protected.
//
// SECURED PDU LAYOUT
//
//   [ Authentic payload ][ Truncated freshness ][ Truncated authenticator ]
//
// The freshness field is optional per message: a message that inherits its
// freshness from another one (as the image chunks inherit theirs from
// IMG_START) carries the authenticator alone.
//
// WHAT THE AUTHENTICATOR COVERS
//
//   MAC = BLAKE2s-keyed( DataID[2] BE || Freshness[4] BE || Authentic payload )
//
// The Data ID is what stops a valid message for one CAN ID being replayed as a
// valid message for another -- without it, every message under one key is
// interchangeable. The FULL freshness value goes into the MAC even though only
// its low bits are transmitted; that is the point of the construction, since
// it means a receiver cannot be fooled by the truncation.
//
// FRESHNESS
//
// A RAMN_SecOC_Freshness_t is a monotonic counter that a group of messages
// shares. The sender increments it; the receiver keeps the highest value it
// has accepted and refuses anything at or below it, which is what makes a
// recorded message useless on replay.
//
// Only the low bits travel, so the receiver rebuilds the full value from its
// own counter (RAMN_SecOC_RxFreshness). This is AUTOSAR's approach and it is
// what keeps the wire cost to one or two bytes instead of four.
//
// KNOWN LIMIT, AND IT IS DELIBERATE
//
// The counter lives in RAM. A receiver that reboots forgets what it has seen
// and re-synchronises to whatever arrives first, so a message recorded before
// the reboot can be replayed after it. Closing that needs the counter
// persisted, or a freshness-sync exchange at startup -- AUTOSAR specifies a
// Freshness Value Manager for exactly this. RAMN_SecOC_RxAccept is the single
// place a persisted counter would be written, so that upgrade lands in one
// function. Within one uptime, replay is fully covered.
//
// REENTRANCY
//
// MAC computation uses a single module-scope hash context, because a BLAKE2s
// context on the stack put ECU A's CAN RX task over its 1 KB budget. So
// RAMN_SecOC_ComputeMac and RAMN_SecOC_CheckMac must be called from ONE task
// per ECU. Today that holds by construction -- ECU A verifies only on the CAN
// RX task, ECU D protects only on the periodic task -- and
// test/host/check_targets.sh measures the chain so the budget cannot quietly
// drift back. A second calling task needs a mutex around these two functions,
// or a per-task context.
//
// PORTABILITY
//
// Depends on nothing but the C standard library and ramn_blake2s.h -- no
// main.h, no HAL, no ramn_config.h. It compiles unchanged on the host test
// runner, on any of the four ECU targets, and on the ESP32 add-on board.
// Key storage is deliberately NOT here; see ramn_secoc_keys.h.

#ifndef INC_RAMN_SECOC_H_
#define INC_RAMN_SECOC_H_

#include <stdint.h>
#include <stddef.h>

// Key length. 16 bytes is the AES-128 length AUTOSAR's profiles assume, kept
// here so a later swap to CMAC needs no key-format change.
#define RAMN_SECOC_KEY_BYTES      16U

// Largest truncated authenticator this module will emit.
#define RAMN_SECOC_MAX_MAC_BYTES  16U

// How far ahead of the receiver's counter a freshness value may jump and still
// be accepted. Covers messages genuinely lost in transit -- a burst dropped by
// a full FDCAN RX FIFO must not desynchronise the link permanently -- while
// keeping an attacker from jumping the counter to the end of its range and
// locking the receiver out of every future message.
#define RAMN_SECOC_FV_WINDOW      1024U

typedef struct
{
	uint32_t txCounter;   // next freshness value the sender will use
	uint32_t rxCounter;   // highest freshness value the receiver has accepted
	uint8_t  rxSynced;    // 0 until the receiver has accepted its first message
} RAMN_SecOC_Freshness_t;

typedef struct
{
	uint16_t                dataId;   // unique per protected message
	uint8_t                 macLen;   // truncated authenticator bytes on the wire
	const uint8_t*          key;      // RAMN_SECOC_KEY_BYTES
	RAMN_SecOC_Freshness_t* fv;       // freshness domain; may be shared
} RAMN_SecOC_Ctx_t;

// Writes ctx->macLen authenticator bytes for this payload at this freshness.
void RAMN_SecOC_ComputeMac(const RAMN_SecOC_Ctx_t* ctx, uint32_t freshness,
                           const uint8_t* payload, uint16_t payloadLen,
                           uint8_t* macOut);

// 1 if macIn matches, 0 otherwise. Compares in constant time: a compare that
// returns early on the first wrong byte lets an attacker find a valid tag one
// byte at a time, which turns a 2^32 problem into a 4*256 one.
uint8_t RAMN_SecOC_CheckMac(const RAMN_SecOC_Ctx_t* ctx, uint32_t freshness,
                            const uint8_t* payload, uint16_t payloadLen,
                            const uint8_t* macIn);

// Consumes and returns the next transmit freshness value.
uint32_t RAMN_SecOC_TxFreshness(RAMN_SecOC_Freshness_t* fv);

// Rebuilds a full freshness value from the low truncBits carried on the wire.
// Returns 1 and writes *fullOut when the result is a plausible next value,
// 0 when it is stale or beyond RAMN_SECOC_FV_WINDOW.
//
// This does NOT advance the receiver's counter: the value has not been
// authenticated yet at this point. Call RAMN_SecOC_RxAccept once the MAC over
// it verifies, and not before -- otherwise anyone can walk the counter forward
// with garbage and lock out the real sender.
uint8_t RAMN_SecOC_RxFreshness(const RAMN_SecOC_Freshness_t* fv, uint32_t truncFv,
                               uint8_t truncBits, uint32_t* fullOut);

// Records a freshness value as accepted, after its MAC verified.
void RAMN_SecOC_RxAccept(RAMN_SecOC_Freshness_t* fv, uint32_t acceptedFv);

// Returns both counters to their startup state.
void RAMN_SecOC_FreshnessInit(RAMN_SecOC_Freshness_t* fv);

#endif /* INC_RAMN_SECOC_H_ */
