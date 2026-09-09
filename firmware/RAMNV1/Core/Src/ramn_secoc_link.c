/*
 * ramn_secoc_link.c
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

// main.h first, deliberately. Every other module reaches it through a header
// that lives beside this .c file; this one's first include is its own header
// in Core/Inc, which would resolve "main.h" relative to THAT directory and
// open the real HAL-dependent copy even in a host build. Including it here,
// from Core/Src, lets the host test runner's stub win. No effect on a firmware
// build, where both names are the same file.
#include "main.h"
#include "ramn_secoc_link.h"

#ifdef ENABLE_IMAGE_SECOC

#include "ramn_secoc_keys.h"
#include "ramn_canfd.h"
#include "ramn_trng.h"

// ---------------------------------------------------------------------------
//   D -> A  0x307  SESSION_REQ        (unauthenticated: it has to be)
//   A -> D  0x308  SESSION_CHALLENGE  nonce_A
//   D -> A  0x309  SESSION_RESPONSE   nonce_D | MAC_root(nonce_A | nonce_D)
//   A -> D  0x30A  SESSION_CONFIRM             MAC_root(nonce_D | nonce_A)
// ---------------------------------------------------------------------------

// The ESTABLISHED session, and the one being NEGOTIATED, deliberately apart.
// SESSION_REQ and SESSION_CHALLENGE cannot be authenticated -- agreeing a key
// is what makes authentication possible -- so anyone on the bus may send them.
// Negotiating into scratch is what makes that harmless: the live session is
// replaced only when a transcript MAC verifies under the provisioned key.
static RAMN_SecOC_Session_t linkSession;
static RAMN_SecOC_Session_t linkPending;

static uint16_t linkFailedCnt   = 0U;
static uint32_t linkGeneration  = 0U;
static uint32_t linkLastTxTick  = 0U;
static RAMN_Bool_t linkTxSent   = False;

#ifdef SECOC_LINK_ROLE_VERIFIER
static uint32_t linkPendingTick = 0U;
#endif

static void SendSessionFrame(uint32_t canId, const uint8_t* payload, uint32_t dlc)
{
	FDCAN_TxHeaderTypeDef h;
	h.Identifier          = canId;
	h.IdType              = FDCAN_STANDARD_ID;
	h.TxFrameType         = FDCAN_DATA_FRAME;
	h.DataLength          = dlc;
	h.BitRateSwitch       = FDCAN_BRS_OFF;
	h.FDFormat            = FDCAN_FD_CAN;
	h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	h.MessageMarker       = 0U;
	h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	RAMN_FDCAN_SendMessage(&h, payload);
}

static void DrawNonce(uint8_t* out)
{
	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i += 4U)
	{
		uint32_t r = RAMN_RNG_Pop32();
		out[i]      = (uint8_t)( r        & 0xFFU);
		out[i + 1U] = (uint8_t)((r >>  8) & 0xFFU);
		out[i + 2U] = (uint8_t)((r >> 16) & 0xFFU);
		out[i + 3U] = (uint8_t)((r >> 24) & 0xFFU);
	}
}

// Commits the negotiated session over the live one. The freshness counters
// restart at zero with it, which is safe only because the key changed too: a
// message recorded under the previous session cannot verify under this one
// however its freshness value compares.
static void AdoptPending(void)
{
	RAMN_SecOC_SESSION_Adopt(&linkPending);
	linkSession = linkPending;
	linkTxSent  = False;
	linkGeneration++;
	RAMN_SecOC_SESSION_Reset(&linkPending);
}

void RAMN_SecOC_LINK_Init(void)
{
	RAMN_SecOC_SESSION_Reset(&linkSession);
	RAMN_SecOC_SESSION_Reset(&linkPending);
	linkFailedCnt  = 0U;
	linkTxSent     = False;
	linkLastTxTick = 0U;
}

uint8_t RAMN_SecOC_LINK_Ready(void)
{
	return (linkSession.state == RAMN_SECOC_SESSION_OK) ? 1U : 0U;
}

const uint8_t* RAMN_SecOC_LINK_Key(void)
{
	return linkSession.key;
}

RAMN_SecOC_Freshness_t* RAMN_SecOC_LINK_Freshness(void)
{
	return &linkSession.fv;
}

uint32_t RAMN_SecOC_LINK_Generation(void)
{
	return linkGeneration;
}

uint16_t RAMN_SecOC_LINK_FailedCount(void)
{
	return linkFailedCnt;
}

#ifdef SECOC_LINK_ROLE_VERIFIER
// The far side asked for a session. Answer with a fresh challenge -- into the
// PENDING session, never the live one.
static void HandleSessionReq(uint32_t tick)
{
	// Rate limit. SESSION_REQ is the unauthenticated message an attacker can
	// flood, so capping the reply rate keeps that to wasted frames rather than
	// a TRNG and bus workout. A handshake already in flight may still be
	// superseded once it has had its chance, otherwise a lost SESSION_RESPONSE
	// would wedge the link until reboot.
	if ((linkTxSent != False) &&
	    ((tick - linkLastTxTick) < (uint32_t)SESSION_REQ_INTERVAL_MS)) return;

	RAMN_SecOC_SESSION_Reset(&linkPending);
	DrawNonce(linkPending.nonceA);
	linkPending.state = RAMN_SECOC_SESSION_PENDING;
	linkPendingTick   = tick;
	linkLastTxTick    = tick;
	linkTxSent        = True;

	SendSessionFrame(SESSION_CAN_ID_CHALLENGE, linkPending.nonceA, FDCAN_DLC_BYTES_8);
}

// The far side answered. Verify against the ROOT key, and only then adopt.
static void HandleSessionResponse(const uint8_t* data, uint8_t dlcLen, uint32_t tick)
{
	if (dlcLen < (uint8_t)(RAMN_SECOC_NONCE_BYTES + RAMN_SECOC_SESSION_MAC_BYTES)) return;
	if (linkPending.state != RAMN_SECOC_SESSION_PENDING) return;

	// A stale half-handshake is not answered: the nonce it was built around is
	// old enough that the far side has almost certainly given up and re-asked.
	if ((tick - linkPendingTick) > (uint32_t)SESSION_PENDING_TIMEOUT_MS)
	{
		RAMN_SecOC_SESSION_Reset(&linkPending);
		linkTxSent = False;
		return;
	}

	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i++) linkPending.nonceD[i] = data[i];

	const uint8_t* rootKey = RAMN_SecOC_KEYS_GetImageKey();
	if (RAMN_SecOC_SESSION_CheckMac(rootKey, (uint16_t)SESSION_CAN_ID_RESPONSE,
	                                linkPending.nonceA, linkPending.nonceD,
	                                &data[RAMN_SECOC_NONCE_BYTES]) == 0U)
	{
		// Whoever sent this does not hold the provisioned key. The live
		// session is untouched, which is the whole point of scratch.
		if (linkFailedCnt < 0xFFFFU) linkFailedCnt++;
		RAMN_SecOC_SESSION_Reset(&linkPending);
		linkTxSent = False;
		return;
	}

	RAMN_SecOC_SESSION_Derive(&linkPending, rootKey);
	AdoptPending();

	// Confirm, so the sender knows to start. Nonces in the opposite order and
	// under a different Data ID, so this cannot be reflected back at us as a
	// response.
	uint8_t confirm[RAMN_SECOC_SESSION_MAC_BYTES];
	RAMN_SecOC_SESSION_Mac(rootKey, (uint16_t)SESSION_CAN_ID_CONFIRM,
	                       linkSession.nonceD, linkSession.nonceA, confirm);
	SendSessionFrame(SESSION_CAN_ID_CONFIRM, confirm, FDCAN_DLC_BYTES_8);
}
#endif /* SECOC_LINK_ROLE_VERIFIER */

#ifdef SECOC_LINK_ROLE_SENDER
// We were challenged. Answer with our own nonce and a MAC over both, proving
// we hold the provisioned key.
static void HandleSessionChallenge(const uint8_t* data, uint8_t dlcLen)
{
	if (dlcLen < RAMN_SECOC_NONCE_BYTES) return;

	RAMN_SecOC_SESSION_Reset(&linkPending);
	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i++) linkPending.nonceA[i] = data[i];
	DrawNonce(linkPending.nonceD);
	linkPending.state = RAMN_SECOC_SESSION_PENDING;

	uint8_t resp[RAMN_SECOC_NONCE_BYTES + RAMN_SECOC_SESSION_MAC_BYTES];
	for (uint8_t i = 0U; i < RAMN_SECOC_NONCE_BYTES; i++) resp[i] = linkPending.nonceD[i];
	RAMN_SecOC_SESSION_Mac(RAMN_SecOC_KEYS_GetImageKey(),
	                       (uint16_t)SESSION_CAN_ID_RESPONSE,
	                       linkPending.nonceA, linkPending.nonceD,
	                       &resp[RAMN_SECOC_NONCE_BYTES]);
	SendSessionFrame(SESSION_CAN_ID_RESPONSE, resp, FDCAN_DLC_BYTES_16);
}

// The verifier confirmed. Checking this is what stops an attacker posing as it
// from leaving us streaming under a key the real far side does not hold -- they
// could not read the traffic either way, but they could deny the link.
static void HandleSessionConfirm(const uint8_t* data, uint8_t dlcLen)
{
	if (dlcLen < RAMN_SECOC_SESSION_MAC_BYTES) return;
	if (linkPending.state != RAMN_SECOC_SESSION_PENDING) return;

	if (RAMN_SecOC_SESSION_CheckMac(RAMN_SecOC_KEYS_GetImageKey(),
	                                (uint16_t)SESSION_CAN_ID_CONFIRM,
	                                linkPending.nonceD, linkPending.nonceA, data) == 0U)
	{
		if (linkFailedCnt < 0xFFFFU) linkFailedCnt++;
		RAMN_SecOC_SESSION_Reset(&linkPending);
		return;
	}

	RAMN_SecOC_SESSION_Derive(&linkPending, RAMN_SecOC_KEYS_GetImageKey());
	AdoptPending();
}

uint8_t RAMN_SecOC_LINK_EnsureSession(uint32_t tick)
{
	if (linkSession.state == RAMN_SECOC_SESSION_OK) return 1U;

	if ((linkTxSent == False) ||
	    ((tick - linkLastTxTick) >= (uint32_t)SESSION_REQ_INTERVAL_MS))
	{
		uint8_t req[4] = {0U, 0U, 0U, 0U};
		SendSessionFrame(SESSION_CAN_ID_REQ, req, FDCAN_DLC_BYTES_4);
		linkLastTxTick = tick;
		linkTxSent     = True;
	}
	return 0U;
}

void RAMN_SecOC_LINK_Drop(void)
{
	RAMN_SecOC_SESSION_Reset(&linkSession);
	RAMN_SecOC_SESSION_Reset(&linkPending);
	linkTxSent = False;
}
#endif /* SECOC_LINK_ROLE_SENDER */

void RAMN_SecOC_LINK_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader,
                                         const uint8_t* data, uint32_t tick)
{
	if ((pHeader == NULL) || (data == NULL)) return;
	if (pHeader->IdType != FDCAN_STANDARD_ID) return;
	if (pHeader->RxFrameType != FDCAN_DATA_FRAME) return;

	uint8_t dlcLen = DLCtoUINT8(pHeader->DataLength);

	// Only the two messages this role is meant to RECEIVE are handled. A
	// verifier that also answered SESSION_CHALLENGE would hand anyone on the
	// bus a transcript-MAC oracle; a sender that answered SESSION_REQ would
	// challenge itself.
#ifdef SECOC_LINK_ROLE_VERIFIER
	if (pHeader->Identifier == SESSION_CAN_ID_REQ)      { HandleSessionReq(tick); return; }
	if (pHeader->Identifier == SESSION_CAN_ID_RESPONSE) { HandleSessionResponse(data, dlcLen, tick); return; }
#endif
#ifdef SECOC_LINK_ROLE_SENDER
	if (pHeader->Identifier == SESSION_CAN_ID_CHALLENGE) { HandleSessionChallenge(data, dlcLen); return; }
	if (pHeader->Identifier == SESSION_CAN_ID_CONFIRM)   { HandleSessionConfirm(data, dlcLen); return; }
#endif
	(void)dlcLen;
	(void)tick;
}

#endif /* ENABLE_IMAGE_SECOC */
