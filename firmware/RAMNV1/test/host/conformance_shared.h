#pragma once
#include <stddef.h>
#include <stdint.h>

/* ramn_telematics.c is compiled into test_telematics_spi.c's translation unit
 * (white-box, to reach the static ProcessESP32Response and RX buffers). A
 * second file cannot #include it again without duplicate symbols, so the
 * driver lives there and the conformance cases call through this. */
void conf_feed(const uint8_t *bytes, size_t n);

/* Defined in test_conformance.c, called from main(). */
void conformance_cases(void);
