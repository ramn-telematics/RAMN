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

/* The delta chunk size is derived in ramn-protocol and copied into ECU D and
   the ESP32 encoder. If any of the three drifts, tiles are truncated silently
   -- ECU D just copies fewer bytes and says nothing. Fail the build instead. */
#if defined(RAMN_DELTA_CHUNK_PAYLOAD) && defined(DELTA_CHUNK_PAYLOAD)
#if RAMN_DELTA_CHUNK_PAYLOAD != DELTA_CHUNK_PAYLOAD
#error "ECU D's delta chunk size disagrees with the protocol's"
#endif
#endif
/* The literal this used to compare against (59) was the pre-SecOC size. The
   number is not a constant of the protocol any more -- it is 64 minus the
   header minus the authenticator -- so check the RELATIONSHIP instead, which
   catches drift in either direction and in either repository. */
#if defined(RAMN_DELTA_CHUNK_PAYLOAD) && defined(RAMN_DELTA_CAN_HEADER) && defined(RAMN_SECOC_MAC_BYTES)
#if (RAMN_DELTA_CHUNK_PAYLOAD + RAMN_DELTA_CAN_HEADER + RAMN_SECOC_MAC_BYTES) != 64
#error "vendored delta geometry does not fill a 64-byte CAN FD frame -- refresh the vectors"
#endif
#endif
#if defined(RAMN_PIPE_CAN_FRAME_PAYLOAD) && defined(RAMN_PIPE_CAN_FRAME_HEADER) && defined(RAMN_SECOC_MAC_BYTES)
#if (RAMN_PIPE_CAN_FRAME_PAYLOAD + RAMN_PIPE_CAN_FRAME_HEADER + RAMN_SECOC_MAC_BYTES) != 64
#error "vendored chunk geometry does not fill a 64-byte CAN FD frame -- refresh the vectors"
#endif
#endif
/* And that the vectors describe the SecOC setting this firmware is built for:
   a header saying 4 MAC bytes against a build with none is silent truncation. */
#if defined(RAMN_SECOC_MAC_BYTES) && (RAMN_SECOC_MAC_BYTES != IMG_SECOC_MAC_BYTES)
#error "vendored vectors and ramn_config.h disagree on the SecOC MAC size"
#endif
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

        /* encoded_tx: the 0xCC poll response this decoder actually receives,
           and byte for byte what the ESP32's encoder is asserted to produce.
           This used to swap the marker and recompute the checksum here, while
           the ESP32's suite did its own fixup in the other direction -- so the
           bytes one end produced were never compared against the bytes the
           other consumed, and a shared mistake in the two fixups would pass
           both suites. Feed the literal bytes instead. */
        conf_feed(v->encoded_tx, v->encoded_len);

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
