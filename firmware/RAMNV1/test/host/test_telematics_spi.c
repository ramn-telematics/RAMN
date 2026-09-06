/* Host tests for the ESP32 -> STM32 poll-response decode path.
 *
 * White-box: this #includes ramn_telematics.c so it can reach the static
 * ProcessESP32Response() and the static RX buffers. Same approach the ESP32
 * repo's image_stream host test uses.
 *
 * The assertion surface is RAMN_FDCAN_SendMessage: fakes.c records every
 * frame the code tries to put on the vehicle bus, so each test states what
 * SHOULD reach the bus for a given set of SPI bytes.
 *
 * Fixtures are BUILT, never hand-written. A hardcoded checksum that happens
 * to be wrong looks exactly like a firmware bug; computing it removes a whole
 * class of false findings.
 */
#include <string.h>

#include "harness.h"
#include "fakes.h"
#include "ramn_telematics.c"

/* ------------------------------------------------------------------ */
/* Fixture builders -- the wire format, expressed once                  */
/* ------------------------------------------------------------------ */

/* CAN poll response as the ESP32 emits it TODAY: no TYPE byte.
   [MSGLEN][0xCC][ID31:24][ID23:16][ID15:8][ID7:0][DLC][FLAGS][DATA..][CHK] */
static size_t build_can_response(uint8_t *out, uint32_t id, const uint8_t *data,
                                 uint8_t dlc, uint8_t flags)
{
    out[0] = (uint8_t)(8 + dlc);
    out[1] = 0xCC;
    out[2] = (uint8_t)(id >> 24); out[3] = (uint8_t)(id >> 16);
    out[4] = (uint8_t)(id >>  8); out[5] = (uint8_t)(id);
    out[6] = dlc;
    out[7] = flags;
    for (uint8_t i = 0; i < dlc; i++) out[8 + i] = data[i];
    uint8_t chk = 0;
    for (uint8_t i = 1; i < (uint8_t)(8 + dlc); i++) chk ^= out[i];
    out[8 + dlc] = chk;
    return (size_t)(9 + dlc);
}

/* Idle response: no message queued. */
static size_t build_idle_response(uint8_t *out)
{
    out[0] = 0x02; out[1] = 0xCC; out[2] = 0x00; out[3] = 0xCC;
    return 4;
}

/* Drive one poll response through the decoder. */
static void feed(const uint8_t *bytes, size_t n)
{
    fake_reset();
    memset(&spiStats, 0, sizeof(spiStats));
    memset(spiRxBufferA, 0, SPI_RX_BUFFER_SIZE);
    memcpy(spiRxBufferA, bytes, n);
    processRxBuffer = spiRxBufferA;
#ifdef TELEMATICS_HAS_STREAM_STATE
    /* Only exists once image streaming landed. Guarded so this same file can
       be compiled against a pre-image-stream revision as a control. */
    streamState = STREAM_IDLE;
#endif
    ProcessESP32Response();
}

/* ------------------------------------------------------------------ */
/* Cases                                                               */
/* ------------------------------------------------------------------ */

static void case_idle_forwards_nothing(void)
{
    h_case_begin("idle poll response forwards nothing");
    uint8_t f[8];
    feed(f, build_idle_response(f));
    CHECK(fake_can_tx_count == 0, "no CAN frame is generated");
    CHECK(spiStats.spiRxEmptyRespCnt == 1, "counted as an empty response");
}

static void case_standard_can_frame(void)
{
    h_case_begin("standard ID 0x100, DLC 8 reaches the bus intact");
    const uint8_t payload[8] = {0xFE,0xED,0xFA,0xCE,0xCA,0xFE,0xBE,0xEF};
    uint8_t f[80];
    feed(f, build_can_response(f, 0x100, payload, 8, 0x00));

    /* A frame IS forwarded today -- it is the CONTENTS that are wrong,
       because the ESP32 omits the TYPE byte the decoder expects at msg[2]
       and every field lands one byte late. */
    CHECK(fake_can_tx_count == 1, "exactly one frame forwarded");
    if (fake_can_tx_count != 1) return;

    CapturedFrame_t *c = &fake_can_tx[0];
    CHECK_BUG(c->header.Identifier == 0x100, "CAN ID is 0x100",
              "reads 0x00010008 -- the HAL truncates that to 0x008 on the bus");
    CHECK(c->header.IdType == FDCAN_STANDARD_ID, "standard ID");
    CHECK_BUG(c->header.TxFrameType == FDCAN_DATA_FRAME, "data frame, not remote",
              "FLAGS is read from a payload byte");
    CHECK_BUG(c->len == 8, "DLC 8", NULL);
    CHECK_BUG(memcmp(c->data, payload, 8) == 0, "payload intact", NULL);
}

static void case_extended_id_does_not_alias_a_message_type(void)
{
    h_case_begin("extended ID 0x01ABCDEF is CAN data, not IMG_START");
    /* msg[2] is ID[31:24] on a CAN frame. Extended IDs reach 0x1FFFFFFF, so
       that byte spans 0x00-0x1F -- which currently overlaps every image type
       code (0x01,0x02,0x03,0x04,0x10,0x11). Moving the type codes above 0x1F
       makes byte 2 self-describing and fixes this by construction. */
    const uint8_t payload[2] = {0xAA, 0xBB};
    uint8_t f[80];
    feed(f, build_can_response(f, 0x01ABCDEF, payload, 2, 0x01));

    CHECK_BUG(fake_can_tx_count == 1 &&
              fake_can_tx[0].header.Identifier == 0x01ABCDEF,
              "extended ID forwarded unchanged",
              "dispatched as IMG_START and emitted CAN 0x300 instead;"
              " ~19% of the extended ID space aliases a type code.");
}

static void case_zero_payload_frames_are_legal(void)
{
    h_case_begin("remote frame (DLC 0) is forwarded, not dropped");
    /* A CAN remote frame carries no payload, so it is always the minimum-size
       frame. The guard must not be one byte too strict -- the same off-by-one
       that cost the ESP32 every remote frame (deviation D-1). */
    uint8_t f[80];
    feed(f, build_can_response(f, 0x200, NULL, 0, 0x02));

    CHECK_BUG(fake_can_tx_count == 1, "remote frame forwarded",
              "`if (msgLen < 9U)` rejects it; the minimum CAN response is"
              " msgLen == 8, so the guard should be < 8.");
}

static void case_short_and_malformed_are_rejected(void)
{
    h_case_begin("malformed responses are rejected, not forwarded");
    uint8_t f[80];

    size_t n = build_can_response(f, 0x123, (const uint8_t*)"\x01\x02", 2, 0x00);
    f[n - 1] ^= 0xFF;                       /* corrupt the checksum */
    feed(f, n);
    CHECK(fake_can_tx_count == 0, "bad checksum forwards nothing");
    CHECK(spiStats.spiRxChecksumErrorCnt == 1, "counted as a checksum error");

    n = build_can_response(f, 0x123, (const uint8_t*)"\x01\x02", 2, 0x00);
    f[1] = 0xAB;                            /* wrong marker */
    feed(f, n);
    CHECK(fake_can_tx_count == 0, "bad marker forwards nothing");

    memset(f, 0, sizeof f);                 /* all zeroes, no marker at all */
    feed(f, 32);
    CHECK(fake_can_tx_count == 0, "empty buffer forwards nothing");
    CHECK(spiStats.spiRxNoRespFoundCnt == 1, "counted as no response found");
}

int main(void)
{
    printf("STM32 telematics SPI host tests\n");
    printf("(assertion surface: frames handed to RAMN_FDCAN_SendMessage)\n");

    case_idle_forwards_nothing();
    case_standard_can_frame();
    case_extended_id_does_not_alias_a_message_type();
    case_zero_payload_frames_are_legal();
    case_short_and_malformed_are_rejected();

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
