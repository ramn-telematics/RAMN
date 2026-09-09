/*
 * ramn_blake2s.h
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

// BLAKE2s (RFC 7693), keyed and unkeyed.
//
// This is the MAC primitive behind RAMN_SecOC. The STM32L552 has a TRNG and a
// CRC unit but NO AES accelerator -- AES and PKA are on the L562, not this
// part -- so a SecOC authenticator has to be computed in software. BLAKE2s is
// the right shape for that job here:
//
//   - It works on 32-bit words, so it maps directly onto the Cortex-M33
//     register file. SipHash's 64-bit rotations cost roughly twice as much on
//     a 32-bit core; SHA-256 is comparable per byte but needs the HMAC
//     wrapper below.
//   - Keyed mode IS a MAC. BLAKE2 is not length-extendable, so unlike SHA-256
//     it needs no HMAC construction -- one pass, one key block, done. That is
//     one fewer moving part to get wrong.
//   - It is small: the whole implementation is one compression function and
//     two constant tables.
//
// AUTOSAR's own SecOC profiles specify AES-128-CMAC. Nothing in SecOC requires
// it -- the authenticator is an abstract PDU-level function -- and swapping
// this module for CMAC later means reimplementing RAMN_SecOC_ComputeMac and
// nothing else. See ramn_secoc.h.

// This header deliberately depends on nothing but the C standard library --
// not main.h, not the HAL, not ramn_config.h. It is a self-contained
// primitive, so it compiles unchanged on the host test runner, on either STM32
// target, and on the ESP32 add-on board if that ever needs to verify too.

#ifndef INC_RAMN_BLAKE2S_H_
#define INC_RAMN_BLAKE2S_H_

#include <stdint.h>
#include <stddef.h>

#define RAMN_BLAKE2S_BLOCKBYTES 64U
#define RAMN_BLAKE2S_OUTBYTES   32U
#define RAMN_BLAKE2S_KEYBYTES   32U

typedef struct
{
	uint32_t h[8];
	uint32_t t[2];                            // 64-bit byte counter
	uint8_t  buf[RAMN_BLAKE2S_BLOCKBYTES];    // partial block
	uint8_t  buflen;
	uint8_t  outlen;
} RAMN_Blake2s_Ctx_t;

// Starts a hash producing outlen bytes (1..32).
// Pass key=NULL/keylen=0 for a plain hash, or a key of 1..32 bytes for a MAC.
void RAMN_BLAKE2S_Init(RAMN_Blake2s_Ctx_t* ctx, uint8_t outlen,
                       const uint8_t* key, uint8_t keylen);

// Absorbs len bytes. May be called any number of times; the caller does not
// need to assemble the message contiguously, which is the whole reason this
// is a streaming API -- RAMN_SecOC feeds it a header and a payload from two
// different buffers and never pays for a copy that joins them.
void RAMN_BLAKE2S_Update(RAMN_Blake2s_Ctx_t* ctx, const uint8_t* in, uint32_t len);

// Writes the outlen bytes fixed at Init and wipes the context.
void RAMN_BLAKE2S_Final(RAMN_Blake2s_Ctx_t* ctx, uint8_t* out);

#endif /* INC_RAMN_BLAKE2S_H_ */
