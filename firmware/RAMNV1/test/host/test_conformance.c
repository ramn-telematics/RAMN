/* Conformance tests: ProcessESP32Response against the golden vectors.
 *
 * ramn_test_vectors.h is GENERATED in the ramn-protocol repository and
 * vendored here -- see VENDOR.md for the pinned revision. Do not hand-edit it.
 *
 * test_telematics_spi.c builds its own fixtures. That catches a decoder that
 * disagrees with itself. These catch a decoder that disagrees with the OTHER
 * END of the link: the vectors come from an implementation written from the
 * spec, in another language, in another repository. A shared misreading of the
 * wire format survives a self-consistent test and dies here.
 *
 * The assertion surface is the same: frames handed to RAMN_FDCAN_SendMessage.
 */
#include <string.h>

#include "harness.h"
#include "fakes.h"
#include "ramn_test_vectors.h"
#include "conformance_shared.h"

/* A stale vendored header means fewer tests run, silently. Make it a build
 * failure instead, naming the family that is missing. */
#if !defined(RAMN_VECTOR_COUNT) || !defined(RAMN_REJECT_VECTOR_COUNT)
#error "vendored ramn_test_vectors.h predates the frame vectors -- refresh it"
#endif
#if !defined(RAMN_ADVERSARIAL_VECTOR_COUNT)
#error "vendored ramn_test_vectors.h predates the adversarial vectors -- refresh it"
#endif
#if !defined(RAMN_MSG_TYPE_MIN) || !defined(RAMN_MAX_ID_HIGH_BYTE)
#error "vendored ramn_test_vectors.h predates the envelope constants -- refresh it"
#endif

void conformance_cases(void)
{
    /* --------------------------------------------------------------- */
    h_case_begin("every golden CAN vector reaches the bus intact");
    for (size_t i = 0; i < RAMN_VECTOR_COUNT; i++) {
        const ramn_vector_t *v = &ramn_vectors[i];

        /* The vectors are the STM32 -> ESP32 direction (marker 0xAA). This
           decoder reads poll responses (0xCC). The body is identical, so swap
           the marker and recompute the checksum over it rather than skipping
           the family -- the fields under test are the same bytes either way. */
        uint8_t f[96];
        memcpy(f, v->encoded, v->encoded_len);
        f[1] = 0xCC;
        uint8_t chk = 0;
        for (size_t k = 1; k < v->encoded_len - 1; k++) chk ^= f[k];
        f[v->encoded_len - 1] = chk;

        conf_feed(f, v->encoded_len);

        if (!CHECK_OK(fake_can_tx_count == 1, v->name)) continue;
        CapturedFrame_t *c = &fake_can_tx[0];
        CHECK(c->header.Identifier == v->can_id, v->name);
        CHECK(c->header.IdType ==
              (v->extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID), v->name);
        CHECK(c->header.TxFrameType ==
              (v->remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME), v->name);
        if (!v->remote && v->dlc > 0) {
            CHECK(c->len == v->dlc, v->name);
            CHECK(memcmp(c->data, v->data, v->dlc) == 0, v->name);
        }
    }

    /* --------------------------------------------------------------- */
    h_case_begin("every malformed golden vector is refused");
    /* Without this the case above passes on a decoder that forwards anything. */
    for (size_t i = 0; i < RAMN_REJECT_VECTOR_COUNT; i++) {
        const ramn_reject_vector_t *v = &ramn_reject_vectors[i];

        /* encoded_tx, built malformed in this direction. Swapping the marker
           here and recomputing the checksum would repair "bad checksum" --
           the checksum covers the marker -- and report a firmware failure
           that is really the test's. */
        conf_feed(v->encoded_tx, v->encoded_tx_len);
        CHECK(fake_can_tx_count == 0, v->name);
    }

    /* --------------------------------------------------------------- */
    h_case_begin("identifiers that look like type codes still reach the bus");
    /* The family covering the collision that put ID 0x008 on the wire for
       every cansend. `encoded` is the 0xCC direction, which is this
       decoder's; the ESP32 runs `encoded_rx`. */
    for (size_t i = 0; i < RAMN_ADVERSARIAL_VECTOR_COUNT; i++) {
        const ramn_adversarial_vector_t *v = &ramn_adversarial_vectors[i];

        conf_feed(v->encoded, v->encoded_len);

        if (!CHECK_OK(fake_can_tx_count == 1, v->name)) continue;
        CHECK(fake_can_tx[0].header.Identifier == v->can_id, v->name);
        CHECK(fake_can_tx[0].header.IdType == FDCAN_EXTENDED_ID, v->name);
    }

    /* --------------------------------------------------------------- */
    h_case_begin("the dispatch rule rests on a real gap");
    CHECK(RAMN_MAX_ID_HIGH_BYTE < RAMN_MSG_TYPE_MIN,
          "no identifier high byte can reach the lowest type code");
    CHECK(RAMN_MSG_TYPE_IMG_START >= RAMN_MSG_TYPE_MIN, "IMG_START clears the floor");
    CHECK(RAMN_MSG_TYPE_IMG_CHUNK >= RAMN_MSG_TYPE_MIN, "IMG_CHUNK clears the floor");
    CHECK(RAMN_MSG_TYPE_IMG_END >= RAMN_MSG_TYPE_MIN, "IMG_END clears the floor");
    CHECK(RAMN_MSG_TYPE_IMG_ABORT >= RAMN_MSG_TYPE_MIN, "IMG_ABORT clears the floor");

    /* The firmware's own constants must equal the protocol's, or the two
       definitions drift and nothing says so. */
    CHECK(RAMN_MSG_TYPE_IMG_START == 0x81U, "IMG_START matches the firmware");
    CHECK(RAMN_MSG_TYPE_IMG_CHUNK == 0x82U, "IMG_CHUNK matches the firmware");
    CHECK(RAMN_MSG_TYPE_IMG_END   == 0x83U, "IMG_END matches the firmware");
    CHECK(RAMN_MSG_TYPE_IMG_ABORT == 0x84U, "IMG_ABORT matches the firmware");
}
