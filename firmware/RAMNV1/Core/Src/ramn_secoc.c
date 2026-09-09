/*
 * ramn_secoc.c
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

#include "ramn_secoc.h"
#include "ramn_blake2s.h"

// Hash scratch, deliberately NOT on the stack.
//
// A BLAKE2s context is ~108 bytes and the digest another 32. On ECU A this
// code runs inside SCREENIMAGE_ProcessRxCANMessage on the CAN RX task, whose
// entire stack is 1 KB, and measuring the chain (test/host/check_targets.sh)
// showed it 320 bytes over budget with these as locals -- a stack overflow on
// real hardware, not a theoretical one.
//
// The cost is that MAC computation is NOT REENTRANT. That is sound here and
// checkable: on ECU A only the CAN RX task verifies, on ECU D only the
// periodic task protects, so no ECU ever computes two MACs at once. Anything
// that later calls SecOC from a second task on the same ECU must either
// serialise on this or give each task its own context -- see the note in
// ramn_secoc.h.
static RAMN_Blake2s_Ctx_t secocHash;
static uint8_t            secocDigest[RAMN_BLAKE2S_OUTBYTES];

void RAMN_SecOC_ComputeMac(const RAMN_SecOC_Ctx_t* ctx, uint32_t freshness,
                           const uint8_t* payload, uint16_t payloadLen,
                           uint8_t* macOut)
{
	uint8_t            hdr[6];
	uint8_t            macLen;

	if ((ctx == NULL) || (macOut == NULL)) return;

	macLen = ctx->macLen;
	if (macLen == 0U) return;
	if (macLen > RAMN_SECOC_MAX_MAC_BYTES) macLen = RAMN_SECOC_MAX_MAC_BYTES;

	// Data ID then the FULL freshness value, both big-endian, then the payload.
	// Fixed-width fields in a fixed order, so no two different (id, fv,
	// payload) triples can ever produce the same MAC input.
	hdr[0] = (uint8_t)((ctx->dataId >> 8) & 0xFFU);
	hdr[1] = (uint8_t)( ctx->dataId       & 0xFFU);
	hdr[2] = (uint8_t)((freshness >> 24) & 0xFFU);
	hdr[3] = (uint8_t)((freshness >> 16) & 0xFFU);
	hdr[4] = (uint8_t)((freshness >>  8) & 0xFFU);
	hdr[5] = (uint8_t)( freshness        & 0xFFU);

	// Ask for a full-width digest and truncate, rather than asking BLAKE2s for
	// a short one. The digest length is bound into the parameter block, so a
	// 4-byte request is a DIFFERENT function from the first 4 bytes of a
	// 32-byte one. Truncating here keeps every macLen consistent with every
	// other, which is what lets one Data ID use 4 bytes and another 8.
	RAMN_BLAKE2S_Init(&secocHash, RAMN_BLAKE2S_OUTBYTES, ctx->key, RAMN_SECOC_KEY_BYTES);
	RAMN_BLAKE2S_Update(&secocHash, hdr, sizeof(hdr));
	if ((payload != NULL) && (payloadLen > 0U))
		RAMN_BLAKE2S_Update(&secocHash, payload, (uint32_t)payloadLen);
	RAMN_BLAKE2S_Final(&secocHash, secocDigest);

	for (uint8_t i = 0U; i < macLen; i++) macOut[i] = secocDigest[i];
	for (uint8_t i = 0U; i < RAMN_BLAKE2S_OUTBYTES; i++) secocDigest[i] = 0U;
}

uint8_t RAMN_SecOC_CheckMac(const RAMN_SecOC_Ctx_t* ctx, uint32_t freshness,
                            const uint8_t* payload, uint16_t payloadLen,
                            const uint8_t* macIn)
{
	uint8_t expected[RAMN_SECOC_MAX_MAC_BYTES];
	uint8_t diff   = 0U;
	uint8_t macLen;

	if ((ctx == NULL) || (macIn == NULL)) return 0U;

	macLen = ctx->macLen;
	if (macLen == 0U) return 0U;
	if (macLen > RAMN_SECOC_MAX_MAC_BYTES) macLen = RAMN_SECOC_MAX_MAC_BYTES;

	RAMN_SecOC_ComputeMac(ctx, freshness, payload, payloadLen, expected);

	// Constant time: accumulate every byte, branch on none of them.
	for (uint8_t i = 0U; i < macLen; i++) diff |= (uint8_t)(expected[i] ^ macIn[i]);

	for (uint8_t i = 0U; i < macLen; i++) expected[i] = 0U;

	return (diff == 0U) ? 1U : 0U;
}

uint32_t RAMN_SecOC_TxFreshness(RAMN_SecOC_Freshness_t* fv)
{
	if (fv == NULL) return 0U;
	fv->txCounter++;
	return fv->txCounter;
}

uint8_t RAMN_SecOC_RxFreshness(const RAMN_SecOC_Freshness_t* fv, uint32_t truncFv,
                               uint8_t truncBits, uint32_t* fullOut)
{
	uint32_t mask;
	uint32_t candidate;

	if ((fv == NULL) || (fullOut == NULL)) return 0U;
	if ((truncBits == 0U) || (truncBits > 32U)) return 0U;

	if (truncBits >= 32U)
	{
		// The whole counter is on the wire; nothing to reconstruct.
		candidate = truncFv;
	}
	else
	{
		mask    = ((uint32_t)1U << truncBits) - 1U;
		truncFv = truncFv & mask;

		// The first message after a receiver reboot has nothing to count from,
		// so it establishes the counter rather than being judged against it.
		// This is the re-synchronisation hole documented in the header.
		if (fv->rxSynced == 0U)
		{
			*fullOut = truncFv;
			return 1U;
		}

		// Rebuild: take the receiver's high bits, splice the transmitted low
		// bits in, and step up one truncation period if that landed at or
		// below where we already are.
		candidate = (fv->rxCounter & ~mask) | truncFv;
		if (candidate <= fv->rxCounter)
		{
			// Would wrap past the top of the counter -- treat as stale rather
			// than folding back to zero, where it would out-rank everything.
			if (candidate > (0xFFFFFFFFUL - (mask + 1U))) return 0U;
			candidate += (mask + 1U);
		}
	}

	if (fv->rxSynced == 0U)
	{
		*fullOut = candidate;
		return 1U;
	}

	// Stale, or so far ahead that accepting it would strand the real sender.
	if (candidate <= fv->rxCounter) return 0U;
	if ((candidate - fv->rxCounter) > (uint32_t)RAMN_SECOC_FV_WINDOW) return 0U;

	*fullOut = candidate;
	return 1U;
}

void RAMN_SecOC_RxAccept(RAMN_SecOC_Freshness_t* fv, uint32_t acceptedFv)
{
	if (fv == NULL) return;

	// Never move backwards, even if a caller passes an older value: the
	// counter's only job is to be a high-water mark.
	if ((fv->rxSynced == 0U) || (acceptedFv > fv->rxCounter))
	{
		fv->rxCounter = acceptedFv;
		fv->rxSynced  = 1U;
	}
}

void RAMN_SecOC_FreshnessInit(RAMN_SecOC_Freshness_t* fv)
{
	if (fv == NULL) return;
	fv->txCounter = 0U;
	fv->rxCounter = 0U;
	fv->rxSynced  = 0U;
}
