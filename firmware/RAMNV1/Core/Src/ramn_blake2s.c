/*
 * ramn_blake2s.c
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

#include "ramn_blake2s.h"

// SHA-256's IV, which BLAKE2s reuses (RFC 7693 section 2.6).
static const uint32_t blake2s_IV[8] =
{
	0x6A09E667UL, 0xBB67AE85UL, 0x3C6EF372UL, 0xA54FF53AUL,
	0x510E527FUL, 0x9B05688CUL, 0x1F83D9ABUL, 0x5BE0CD19UL
};

// Message word permutation, 10 rounds (RFC 7693 section 2.7).
static const uint8_t blake2s_sigma[10][16] =
{
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
	{ 14, 10,  4,  8,  9, 15, 13,  6,  1, 12,  0,  2, 11,  7,  5,  3 },
	{ 11,  8, 12,  0,  5,  2, 15, 13, 10, 14,  3,  6,  7,  1,  9,  4 },
	{  7,  9,  3,  1, 13, 12, 11, 14,  2,  6,  5, 10,  4,  0, 15,  8 },
	{  9,  0,  5,  7,  2,  4, 10, 15, 14,  1, 11, 12,  6,  8,  3, 13 },
	{  2, 12,  6, 10,  0, 11,  8,  3,  4, 13,  7,  5, 15, 14,  1,  9 },
	{ 12,  5,  1, 15, 14, 13,  4, 10,  0,  7,  6,  3,  9,  2,  8, 11 },
	{ 13, 11,  7, 14, 12,  1,  3,  9,  5,  0, 15,  4,  8,  6,  2, 10 },
	{  6, 15, 14,  9, 11,  3,  0,  8, 12,  2, 13,  7,  1,  4, 10,  5 },
	{ 10,  2,  8,  4,  7,  6,  1,  5, 15, 11,  9, 14,  3, 12, 13,  0 }
};

static uint32_t rotr32(uint32_t w, uint8_t c)
{
	return (w >> c) | (w << (32U - c));
}

static uint32_t load32le(const uint8_t* src)
{
	return  ((uint32_t)src[0])        |
	       (((uint32_t)src[1]) <<  8) |
	       (((uint32_t)src[2]) << 16) |
	       (((uint32_t)src[3]) << 24);
}

static void store32le(uint8_t* dst, uint32_t w)
{
	dst[0] = (uint8_t)(w         & 0xFFU);
	dst[1] = (uint8_t)((w >>  8) & 0xFFU);
	dst[2] = (uint8_t)((w >> 16) & 0xFFU);
	dst[3] = (uint8_t)((w >> 24) & 0xFFU);
}

#define B2S_G(a, b, c, d, x, y)          \
	do {                                 \
		a = a + b + (x);                 \
		d = rotr32(d ^ a, 16U);          \
		c = c + d;                       \
		b = rotr32(b ^ c, 12U);          \
		a = a + b + (y);                 \
		d = rotr32(d ^ a,  8U);          \
		c = c + d;                       \
		b = rotr32(b ^ c,  7U);          \
	} while (0)

// The compression function. "last" marks the final block, which inverts v[14].
static void blake2s_compress(RAMN_Blake2s_Ctx_t* ctx, const uint8_t block[RAMN_BLAKE2S_BLOCKBYTES],
                             uint8_t last)
{
	// v[16] is the whole working state and cannot be avoided. The message
	// words deliberately are NOT expanded into an m[16] alongside it: this
	// runs on ECU A's CAN RX task, whose entire stack is 1 KB, and 64 bytes
	// there is worth more than the handful of instructions that reloading
	// each word from the block costs. See M() below.
	uint32_t v[16];

#define M(i) load32le(&block[(uint32_t)(i) * 4U])

	for (uint8_t i = 0U; i < 8U; i++)
	{
		v[i]      = ctx->h[i];
		v[i + 8U] = blake2s_IV[i];
	}

	v[12] ^= ctx->t[0];
	v[13] ^= ctx->t[1];
	if (last != 0U) v[14] = ~v[14];

	for (uint8_t r = 0U; r < 10U; r++)
	{
		const uint8_t* s = blake2s_sigma[r];
		B2S_G(v[0], v[4], v[ 8], v[12], M(s[ 0]), M(s[ 1]));
		B2S_G(v[1], v[5], v[ 9], v[13], M(s[ 2]), M(s[ 3]));
		B2S_G(v[2], v[6], v[10], v[14], M(s[ 4]), M(s[ 5]));
		B2S_G(v[3], v[7], v[11], v[15], M(s[ 6]), M(s[ 7]));
		B2S_G(v[0], v[5], v[10], v[15], M(s[ 8]), M(s[ 9]));
		B2S_G(v[1], v[6], v[11], v[12], M(s[10]), M(s[11]));
		B2S_G(v[2], v[7], v[ 8], v[13], M(s[12]), M(s[13]));
		B2S_G(v[3], v[4], v[ 9], v[14], M(s[14]), M(s[15]));
	}

	for (uint8_t i = 0U; i < 8U; i++) ctx->h[i] ^= v[i] ^ v[i + 8U];

#undef M
}

void RAMN_BLAKE2S_Init(RAMN_Blake2s_Ctx_t* ctx, uint8_t outlen,
                       const uint8_t* key, uint8_t keylen)
{
	if (outlen == 0U || outlen > RAMN_BLAKE2S_OUTBYTES) outlen = RAMN_BLAKE2S_OUTBYTES;
	if (key == NULL) keylen = 0U;
	if (keylen > RAMN_BLAKE2S_KEYBYTES) keylen = RAMN_BLAKE2S_KEYBYTES;

	for (uint8_t i = 0U; i < 8U; i++) ctx->h[i] = blake2s_IV[i];

	// Parameter block, folded into h[0]: digest length, key length, fanout 1,
	// depth 1. Everything else is zero for a sequential hash.
	ctx->h[0] ^= 0x01010000UL ^ ((uint32_t)keylen << 8) ^ (uint32_t)outlen;

	ctx->t[0]   = 0U;
	ctx->t[1]   = 0U;
	ctx->buflen = 0U;
	ctx->outlen = outlen;
	for (uint8_t i = 0U; i < RAMN_BLAKE2S_BLOCKBYTES; i++) ctx->buf[i] = 0U;

	// In keyed mode the key, zero-padded to a full block, IS the first message
	// block. That is what makes BLAKE2 a MAC without an HMAC wrapper.
	//
	// Built directly in the context's own block buffer rather than in a
	// 64-byte local: ctx->buf is already zeroed above and is exactly the
	// staging area Update would have copied the key into anyway, so the local
	// was 64 bytes of stack bought for nothing.
	if (keylen > 0U)
	{
		for (uint8_t i = 0U; i < keylen; i++) ctx->buf[i] = key[i];
		ctx->buflen = RAMN_BLAKE2S_BLOCKBYTES;
	}
}

void RAMN_BLAKE2S_Update(RAMN_Blake2s_Ctx_t* ctx, const uint8_t* in, uint32_t len)
{
	if ((in == NULL) || (len == 0U)) return;

	while (len > 0U)
	{
		// A full buffer is only compressed once we know more input follows.
		// BLAKE2 finalisation must mark the LAST block, so the final block can
		// never be flushed here -- it is held back for RAMN_BLAKE2S_Final.
		if (ctx->buflen == RAMN_BLAKE2S_BLOCKBYTES)
		{
			ctx->t[0] += RAMN_BLAKE2S_BLOCKBYTES;
			if (ctx->t[0] < RAMN_BLAKE2S_BLOCKBYTES) ctx->t[1]++;
			blake2s_compress(ctx, ctx->buf, 0U);
			ctx->buflen = 0U;
		}

		uint32_t space = (uint32_t)(RAMN_BLAKE2S_BLOCKBYTES - ctx->buflen);
		uint32_t take  = (len < space) ? len : space;
		for (uint32_t i = 0U; i < take; i++) ctx->buf[ctx->buflen + i] = in[i];
		ctx->buflen = (uint8_t)(ctx->buflen + take);
		in         += take;
		len        -= take;
	}
}

void RAMN_BLAKE2S_Final(RAMN_Blake2s_Ctx_t* ctx, uint8_t* out)
{
	uint8_t hash[RAMN_BLAKE2S_OUTBYTES];

	ctx->t[0] += ctx->buflen;
	if (ctx->t[0] < ctx->buflen) ctx->t[1]++;

	// Zero-pad the trailing partial block, then compress it as the last one.
	for (uint8_t i = ctx->buflen; i < RAMN_BLAKE2S_BLOCKBYTES; i++) ctx->buf[i] = 0U;
	blake2s_compress(ctx, ctx->buf, 1U);

	for (uint8_t i = 0U; i < 8U; i++) store32le(&hash[i * 4U], ctx->h[i]);
	for (uint8_t i = 0U; i < ctx->outlen; i++) out[i] = hash[i];

	// Do not leave key-derived state on the stack of whoever called us.
	for (uint8_t i = 0U; i < RAMN_BLAKE2S_OUTBYTES; i++) hash[i] = 0U;
	for (uint8_t i = 0U; i < RAMN_BLAKE2S_BLOCKBYTES; i++) ctx->buf[i] = 0U;
	for (uint8_t i = 0U; i < 8U; i++) ctx->h[i] = 0U;
	ctx->buflen = 0U;
}
