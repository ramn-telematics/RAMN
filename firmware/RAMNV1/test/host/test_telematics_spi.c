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
#include "conformance_shared.h"

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

/* The conformance cases live in test_conformance.c but cannot include
   ramn_telematics.c a second time, so they drive the decoder through this. */
void conf_feed(const uint8_t *bytes, size_t n) { feed(bytes, n); }

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
       and every field lands one byte late. Fixed: byte 2 is ID[31:24] and
       the dispatch tests it against the identifier ceiling. */
    CHECK(fake_can_tx_count == 1, "exactly one frame forwarded");
    if (fake_can_tx_count != 1) return;

    CapturedFrame_t *c = &fake_can_tx[0];
    CHECK(c->header.Identifier == 0x100, "CAN ID is 0x100");
    CHECK(c->header.IdType == FDCAN_STANDARD_ID, "standard ID");
    CHECK(c->header.TxFrameType == FDCAN_DATA_FRAME, "data frame, not remote");
    CHECK(c->len == 8, "DLC 8");
    CHECK(memcmp(c->data, payload, 8) == 0, "payload intact");
}

static void case_extended_id_does_not_alias_a_message_type(void)
{
    h_case_begin("extended ID 0x01ABCDEF is CAN data, not IMG_START");
    /* msg[2] is ID[31:24] on a CAN frame. Extended IDs reach 0x1FFFFFFF, so
       that byte spans 0x00-0x1F. The image type codes used to live in that
       range (0x01,0x02,0x03,0x04,0x10,0x11) and this frame was dispatched as
       IMG_START. They now start at 0x81, above the ceiling, so byte 2 is
       self-describing and this holds by construction rather than by luck. */
    const uint8_t payload[2] = {0xAA, 0xBB};
    uint8_t f[80];
    feed(f, build_can_response(f, 0x01ABCDEF, payload, 2, 0x01));

    CHECK(fake_can_tx_count == 1 &&
          fake_can_tx[0].header.Identifier == 0x01ABCDEF,
          "extended ID forwarded unchanged");
}

static void case_zero_payload_frames_are_legal(void)
{
    h_case_begin("remote frame (DLC 0) is forwarded, not dropped");
    /* A CAN remote frame carries no payload, so it is always the minimum-size
       frame. The guard must not be one byte too strict -- the same off-by-one
       that cost the ESP32 every remote frame (deviation D-1). It was `< 9U`
       here, which assumed a type byte that a CAN response never carries. */
    uint8_t f[80];
    feed(f, build_can_response(f, 0x200, NULL, 0, 0x02));

    CHECK(fake_can_tx_count == 1, "remote frame forwarded");
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

#ifdef TELEMATICS_HAS_STREAM_STATE
/* Feed the SAME buffer twice, as the ESP32 does when it re-presents a staged
   response ECU D's write did not acknowledge. feed() resets the fixture but not
   the dedupe fingerprint, which is exactly the state under test. */
static void feed_twice(const uint8_t *bytes, size_t n, int *first, int *second)
{
    feed(bytes, n);
    *first = fake_can_tx_count;
    feed(bytes, n);
    *second = fake_can_tx_count;
}

/* IMG_START as the ESP32 stages it: 13 bytes, checksum over 1..11. */
static size_t build_img_start(uint8_t *out, uint16_t w, uint16_t h,
                              uint16_t chunks, uint8_t scale)
{
    memset(out, 0, 16);
    out[0]  = 0x0C;
    out[1]  = 0xCC;
    out[2]  = RAMN_MSG_TYPE_IMG_START;
    out[3]  = (uint8_t)(w & 0xFF);      out[4]  = (uint8_t)(w >> 8);
    out[5]  = (uint8_t)(h & 0xFF);      out[6]  = (uint8_t)(h >> 8);
    out[7]  = (uint8_t)(chunks & 0xFF); out[8]  = (uint8_t)(chunks >> 8);
    out[9]  = 0; out[10] = 0;
    out[11] = scale;
    uint8_t chk = 0;
    for (uint8_t i = 1; i <= 11; i++) chk ^= out[i];
    out[12] = chk;
    return 13;
}

static void case_a_re_presented_image_transaction_is_forwarded_once(void)
{
    h_case_begin("an image transaction the ESP32 presents twice is forwarded once");
    /* "Re-present when unsure" is the right behaviour for a link that cannot
       retransmit, and it is only safe because the receiver ignores what it has
       already seen. ECU A's chunk sequence gate does that for 0x301 -- its
       duplicate counter fires on hardware, flags=0x10 -- but IMG_START and
       IMG_END carry no sequence and both change state.

       A second IMG_START zeroes kfExpectedSeq and the counters mid-frame and
       queues a second START entry, which ECU A's latest-frame-wins skip then
       reads as "this frame is stale". A second IMG_END finds imgState already
       IMG_SHOWN and comes back IMG_ACK_LATE, with every chunk behind it
       refused out-of-state. Hardware signature: st=3 decoded=0, then
       flags=0x01/0x04/0x05 and a partial decode, all with nothing lost.

       The repeat is a property of the TRANSACTION, so it is caught here, in
       the one place that can see a whole one. */
    uint8_t f[16];
    size_t  n = build_img_start(f, 60, 60, 25, 4);
    int     first = 0, second = 0;

    lastImageRespFingerprint = 0;
    feed_twice(f, n, &first, &second);

    CHECK(first == 1, "the first presentation forwards IMG_START to the bus");
    CHECK(second == 0, "the second forwards nothing at all");
    CHECK(spiStats.spiRxRepeatRespCnt == 1, "and is counted as a repeat");
}

static void case_a_repeated_plain_can_frame_is_still_forwarded(void)
{
    h_case_begin("a plain CAN frame repeated identically is still forwarded");
    /* A periodic frame polled faster than it changes repeats byte for byte and
       legitimately. Deduplicating those would be dropping real gateway
       traffic, so only image traffic -- which carries sequence numbers, and so
       cannot repeat by accident -- is deduplicated. */
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t f[32];
    size_t  n = build_can_response(f, 0x123, data, 8, 0);
    int     first = 0, second = 0;

    lastImageRespFingerprint = 0;
    feed_twice(f, n, &first, &second);

    CHECK(first == 1, "the first copy reaches the bus");
    CHECK(second == 1, "and so does the second");
}

static void case_a_different_image_transaction_is_not_mistaken_for_a_repeat(void)
{
    h_case_begin("a different image transaction is not mistaken for a repeat");
    uint8_t a[16], b[16];
    size_t  na = build_img_start(a, 60, 60, 25, 4);
    size_t  nb = build_img_start(b, 60, 60, 26, 4);   /* one chunk more */

    lastImageRespFingerprint = 0;
    feed(a, na);
    int first = fake_can_tx_count;
    feed(b, nb);
    int second = fake_can_tx_count;

    CHECK(first == 1, "the first frame goes out");
    CHECK(second == 1, "and so does the next, which differs by one byte");
    CHECK(spiStats.spiRxRepeatRespCnt == 0, "nothing is counted as a repeat");
}

static void case_an_img_ack_late_does_not_end_the_keyframe_wait(void)
{
    h_case_begin("an IMG_ACK_LATE does not end the keyframe wait");
    /* st=3 means "0x302 reached me but I was not receiving a keyframe" -- the
       opposite of a completion. Taken as one it ends the wait early and
       reports its own arrival as the frame's paint time, which is why a log of
       genuine 70-90 ms paints was salted with 7-12 ms ones measuring nothing. */
    fake_reset();
    fake_tick     = 400000u;
    streamState   = KEYFRAME_SENT;
    kfAckReceived = False;
    kfAckWaitTick = (uint32_t)xTaskGetTickCount();

    FDCAN_RxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof hdr);
    hdr.Identifier = IMG_CAN_ID_ACK;
    hdr.DataLength = FDCAN_DLC_BYTES_8;
    uint8_t late[8] = {0x03, 0, 0, 0, 0, 0, 0, 0};
    RAMN_TELEMATICS_ProcessImageACK(&hdr, late, 0u);
    CHECK(kfAckReceived == False, "a late ACK is not taken as the frame completing");

    uint8_t done[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};
    RAMN_TELEMATICS_ProcessImageACK(&hdr, done, 0u);
    CHECK(kfAckReceived == True, "but the real one is");
}

static void case_the_keyframe_ack_wait_is_measured_against_its_own_clock(void)
{
    h_case_begin("the keyframe ACK wait is measured against the clock it was armed with");
    /* kfAckWaitTick is stamped with xTaskGetTickCount() -- real time -- in
       ProcessESP32Response. The wait used to be compared against
       RAMN_TELEMATICS_Update's `tick` argument, which is xLastWakeTime:
       vTaskDelayUntil advances that by exactly one period per iteration, so it
       falls permanently behind real time whenever the loop overruns, and this
       loop overruns whenever the SPI debug output is on. Subtracting a real
       stamp from a lagging clock is negative, wraps to about 4.29 billion, and
       clears the 2 s limit on the FIRST call after the wait was armed.

       The cost was not the missing diagnostic. The timeout branch sets
       currentPollIntervalMs back to SPI_POLL_INTERVAL_MS, so every keyframe
       ended by dropping the ESP32 poll rate from 1 ms to 50 ms -- and the next
       keyframe, which needs one poll per 15 chunks, was then fetched at the
       idle rate. Same defect ECU A's image screen had, same fix: read the
       clock the stamp came from, and compare as a signed difference. */
    fake_reset();
    fake_tick             = 500000u;
    streamState           = KEYFRAME_SENT;
    kfAckReceived         = False;
    kfAckWaitTick         = (uint32_t)xTaskGetTickCount();
    currentPollIntervalMs = 1u;

    /* The real entry point, driven the way the overrunning periodic task drives
       it: a `tick` argument far behind the clock the wait was armed with. This
       is the exact call that used to end the wait instantly. */
    RAMN_TELEMATICS_Update(0u);
    CHECK(streamState == KEYFRAME_SENT,
          "a tick argument behind real time does not end the wait");
    CHECK(currentPollIntervalMs == 1u, "so the fast poll rate is kept for the next keyframe");

    UpdateStreamTimeouts();
    CHECK(streamState == KEYFRAME_SENT, "and neither does no elapsed time");

    /* And the stamp itself ahead of the clock -- what a lagging comparison
       clock looks like from the inside. Unsigned, this is ~4.29 billion. */
    kfAckWaitTick = (uint32_t)xTaskGetTickCount() + 100u;
    UpdateStreamTimeouts();
    CHECK(streamState == KEYFRAME_SENT, "nor does a stamp taken ahead of the clock");
    kfAckWaitTick = (uint32_t)xTaskGetTickCount();

    fake_tick += KF_ACK_TIMEOUT_MS - 1u;
    UpdateStreamTimeouts();
    CHECK(streamState == KEYFRAME_SENT, "nor does one tick under the limit");

    fake_tick += 1u;
    UpdateStreamTimeouts();
    CHECK(streamState == STREAM_IDLE, "a real 2 s wait does time out");
    CHECK(currentPollIntervalMs == SPI_POLL_INTERVAL_MS, "and returns to the idle poll rate");
}

static void case_an_ack_that_arrives_reports_how_long_it_took(void)
{
    h_case_begin("an ACK that arrives reports the measured wait, not the limit");
    /* "TIMEOUT after 2000ms" printed the CONSTANT. A mismatched-clock
       comparison that fires instantly prints exactly the same line as a
       genuine two-second wait, which is indistinguishable in a log -- and
       telling those apart is the whole question when keyframes look slow. */
    fake_reset();
    fake_tick             = 900000u;
    streamState           = KEYFRAME_SENT;
    kfAckWaitTick         = (uint32_t)xTaskGetTickCount();
    kfAckReceived         = True;
    kfAckLatencyPrint     = False;
    fake_tick += 37u;

    UpdateStreamTimeouts();
    CHECK(streamState == STREAM_IDLE, "the ACK ends the wait");
    CHECK(kfAckLatencyPrint != False, "and the round trip is queued for printing");
    CHECK(kfAckLatencyMs == 37u, "as the time actually measured");
}

static void case_the_delta_idle_timeout_uses_the_same_clock(void)
{
    h_case_begin("the delta idle timeout uses the same clock as its stamp");
    /* Same mismatch, same consequence: DELTA_ACTIVE was torn down between
       tiles, dropping the poll rate mid-stream. */
    fake_reset();
    fake_tick             = 700000u;
    streamState           = DELTA_ACTIVE;
    lastDeltaActivityTick = (uint32_t)xTaskGetTickCount();
    currentPollIntervalMs = 1u;

    UpdateStreamTimeouts();
    CHECK(streamState == DELTA_ACTIVE, "a fresh tile does not end the stream");

    fake_tick += DELTA_IDLE_TIMEOUT_MS;
    UpdateStreamTimeouts();
    CHECK(streamState == STREAM_IDLE, "but a real idle period does");
}
#endif

int main(void)
{
    printf("STM32 telematics SPI host tests\n");
    printf("(assertion surface: frames handed to RAMN_FDCAN_SendMessage)\n");

    case_idle_forwards_nothing();
    case_standard_can_frame();
    case_extended_id_does_not_alias_a_message_type();
    case_zero_payload_frames_are_legal();
    case_short_and_malformed_are_rejected();
#ifdef TELEMATICS_HAS_STREAM_STATE
    case_a_re_presented_image_transaction_is_forwarded_once();
    case_a_repeated_plain_can_frame_is_still_forwarded();
    case_a_different_image_transaction_is_not_mistaken_for_a_repeat();
    case_an_img_ack_late_does_not_end_the_keyframe_wait();
    case_the_keyframe_ack_wait_is_measured_against_its_own_clock();
    case_an_ack_that_arrives_reports_how_long_it_took();
    case_the_delta_idle_timeout_uses_the_same_clock();
#endif

    printf("\n-- against the golden vectors from ramn-protocol --\n");
    conformance_cases();

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
