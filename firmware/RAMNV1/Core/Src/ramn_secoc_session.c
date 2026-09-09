/*
 * ramn_secoc_session.c
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

#include "ramn_secoc_session.h"
#include "ramn_blake2s.h"

// Domain separation for the KDF. Versioned so that a future change to what
// goes into the derivation cannot silently agree with an ECU running the old
// scheme -- they would derive different keys and fail closed, which is the
// outcome you want from a version mismatch.
static const char secocSessionLabel[] = "RAMN-SecOC-session-v1";

// Scratch, module scope for the same reason as in ramn_secoc.c: this runs
// inside SCREENIMAGE_ProcessRxCANMessage on ECU A's 1 KB CAN RX task, and
// check_targets.sh measured the handshake chain 64 bytes over budget with a
// BLAKE2s context and a transcript buffer as locals. It is DEEPER than the
// image path -- two extra frames above the same hash -- so it is the chain
// that actually sets the budget.
//
// Inherits the non-reentrancy contract documented in ramn_secoc.h: one calling
// task per ECU.
static RAMN_Blake2s_Ctx_t sessionHash;
static uint8_t            sessionDigest[RAMN_BLAKE2S_OUTBYTES];
static uint8_t            sessionTranscript[2U * RAMN_SECOC_NONCE_BYTES];

// Lays the two nonces out in a fixed order for the transcript MAC. Which one
// goes first is what distinguishes a response from a confirm.
static void FillTranscript(const uint8_t* first, const uint8_t* second)
{
	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i++)
	{
		sessionTranscript[i]                          = first[i];
		sessionTranscript[RAMN_SECOC_NONCE_BYTES + i] = second[i];
	}
}

static void SessionCtx(RAMN_SecOC_Ctx_t* ctx, uint16_t dataId, const uint8_t* rootKey)
{
	ctx->dataId = dataId;
	ctx->macLen = RAMN_SECOC_SESSION_MAC_BYTES;
	ctx->key    = rootKey;
	ctx->fv     = NULL;   // not consulted by ComputeMac; freshness is passed in
}

void RAMN_SecOC_SESSION_Reset(RAMN_SecOC_Session_t* s)
{
	if (s == NULL) return;

	s->state = RAMN_SECOC_SESSION_NONE;
	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i++) { s->nonceA[i] = 0U; s->nonceD[i] = 0U; }
	for (uint8_t i = 0U; i < RAMN_SECOC_KEY_BYTES;   i++) s->key[i] = 0U;
	RAMN_SecOC_FreshnessInit(&s->fv);
}

void RAMN_SecOC_SESSION_Derive(RAMN_SecOC_Session_t* s, const uint8_t* rootKey)
{
	if ((s == NULL) || (rootKey == NULL)) return;

	// Keyed by the root, over a label and both nonces in a fixed order. The
	// label makes this derivation distinct from every other use of the same
	// key -- notably from the transcript MACs below, which are computed under
	// the same root and must not be able to collide with a session key.
	RAMN_BLAKE2S_Init(&sessionHash, RAMN_BLAKE2S_OUTBYTES, rootKey, RAMN_SECOC_KEY_BYTES);
	RAMN_BLAKE2S_Update(&sessionHash, (const uint8_t*)secocSessionLabel,
	                    (uint32_t)(sizeof(secocSessionLabel) - 1U));
	RAMN_BLAKE2S_Update(&sessionHash, s->nonceA, RAMN_SECOC_NONCE_BYTES);
	RAMN_BLAKE2S_Update(&sessionHash, s->nonceD, RAMN_SECOC_NONCE_BYTES);
	RAMN_BLAKE2S_Final(&sessionHash, sessionDigest);

	for (uint8_t i = 0U; i < RAMN_SECOC_KEY_BYTES;   i++) s->key[i] = sessionDigest[i];
	for (uint8_t i = 0U; i < RAMN_BLAKE2S_OUTBYTES; i++) sessionDigest[i] = 0U;
}

void RAMN_SecOC_SESSION_Adopt(RAMN_SecOC_Session_t* s)
{
	if (s == NULL) return;

	// Every session counts from zero. It can, because the key is new: a
	// freshness value from a previous session cannot be replayed into this one
	// even though the numbers repeat, since the authenticator over it was
	// computed under a key that no longer exists.
	RAMN_SecOC_FreshnessInit(&s->fv);
	s->state = RAMN_SECOC_SESSION_OK;
}

void RAMN_SecOC_SESSION_Mac(const uint8_t* rootKey, uint16_t dataId,
                            const uint8_t* first, const uint8_t* second,
                            uint8_t* macOut)
{
	RAMN_SecOC_Ctx_t ctx;

	if ((rootKey == NULL) || (first == NULL) || (second == NULL) || (macOut == NULL)) return;

	FillTranscript(first, second);
	SessionCtx(&ctx, dataId, rootKey);

	// Freshness 0: these messages carry their own, in the nonces. A counter
	// would be circular here anyway, since agreeing one is what the handshake
	// is for.
	//
	// The Data ID is the CAN ID carrying this authenticator, which is what
	// stops a SESSION_RESPONSE being reflected back as a SESSION_CONFIRM: the
	// two are computed under different Data IDs, and over the nonces in
	// opposite orders.
	RAMN_SecOC_ComputeMac(&ctx, 0U, sessionTranscript,
	                      (uint16_t)sizeof(sessionTranscript), macOut);
}

uint8_t RAMN_SecOC_SESSION_CheckMac(const uint8_t* rootKey, uint16_t dataId,
                                    const uint8_t* first, const uint8_t* second,
                                    const uint8_t* macIn)
{
	RAMN_SecOC_Ctx_t ctx;
	uint8_t          expected[RAMN_SECOC_SESSION_MAC_BYTES];
	uint8_t          diff = 0U;

	if ((rootKey == NULL) || (first == NULL) || (second == NULL) || (macIn == NULL)) return 0U;

	// Computes directly rather than calling RAMN_SecOC_SESSION_Mac. That looks
	// like duplication and is not: this is the deepest chain on ECU A's 1 KB
	// CAN RX task, and the extra frame put it over budget.
	FillTranscript(first, second);
	SessionCtx(&ctx, dataId, rootKey);
	RAMN_SecOC_ComputeMac(&ctx, 0U, sessionTranscript,
	                      (uint16_t)sizeof(sessionTranscript), expected);

	for (uint8_t i = 0U; i < RAMN_SECOC_SESSION_MAC_BYTES; i++)
		diff |= (uint8_t)(expected[i] ^ macIn[i]);

	for (uint8_t i = 0U; i < RAMN_SECOC_SESSION_MAC_BYTES; i++) expected[i] = 0U;

	return (diff == 0U) ? 1U : 0U;
}
