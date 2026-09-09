/* Host-test stand-in for ramn_trng.h.
 *
 * The real header pulls in main.h and the HAL for the RNG handle type, none of
 * which the modules under test need -- they only ever pop bytes. The fake in
 * fakes.c is a deterministic counter, NOT random: a test that has to predict
 * the nonce ECU A will choose could not do so against a real TRNG, and
 * fake_rng_set lets a case pin it. */
#pragma once
#include <stdint.h>

uint8_t  RAMN_RNG_Pop8(void);
uint16_t RAMN_RNG_Pop16(void);
uint32_t RAMN_RNG_Pop32(void);

/* Test control: force the next values Pop32 will return. */
void fake_rng_set(uint32_t seed);
