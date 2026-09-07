/* Host tests for ECU A's image receive path.
 *
 * White-box: this #includes ramn_screen_image.c so it can reach the static
 * SCREENIMAGE_ProcessRxCANMessage / SCREENIMAGE_Update and the static state.
 * Same approach as the ECU D suite next door.
 *
 * The assertion surface is RAMN_SPI_WriteImageChunk -- exactly the pixel bytes
 * that would reach the ST7789. Each test says what SHOULD land on the panel
 * for a given set of CAN frames.
 *
 * Fixtures are BUILT, never hand-written.
 */
#include <string.h>

#include "harness.h"
#include "fakes_screen.h"
#include "fakes.h"
#include "ramn_test_vectors.h"

/* ENABLE_SCREEN comes from the Makefile (CFLAGS_A), as it does in the real build. */
#include "../../Core/Src/ramn_screen_image.c"

#if !defined(RAMN_RLE_VECTOR_COUNT) || !defined(RAMN_PIPE_VECTOR_COUNT)
#error "vendored ramn_test_vectors.h predates the RLE or pipeline vectors -- refresh it"
#endif
/* A header whose macros all exist but hold old values walks straight past an
   #ifdef guard. This one carried RAMN_PIPE_SPI_CHUNK_PAYLOAD 64 after the
   protocol moved to 61, so this suite sent 64-byte payloads, the 0x301
   handler clamped each to 61, and three bytes of every chunk vanished --
   presenting as a decoder fault for as long as it took to look. Check the
   relationship the protocol guarantees, not just that the names are defined. */
#if RAMN_PIPE_SPI_CHUNK_PAYLOAD != RAMN_PIPE_CAN_FRAME_PAYLOAD
#error "vendored vectors predate the 61-byte chunk: one chunk must be one CAN frame"
#endif

/* ------------------------------------------------------------------ */
/* Fixture builders                                                     */
/* ------------------------------------------------------------------ */

static void feed_can(uint32_t id, const uint8_t *data, uint8_t byteLen, uint32_t tick)
{
    FDCAN_RxHeaderTypeDef h;
    memset(&h, 0, sizeof(h));
    h.Identifier = id;
    /* The bus always sends DLC 64 on image frames regardless of content --
       that is why REAL_LEN exists. Mirror it rather than the true length. */
    h.DataLength = (id == IMG_CAN_ID_DATA) ? FDCAN_DLC_BYTES_64 : FDCAN_DLC_BYTES_12;
    uint8_t buf[64];
    memset(buf, 0, sizeof buf);
    memcpy(buf, data, byteLen);
    SCREENIMAGE_ProcessRxCANMessage(&h, buf, tick);
}

/* IMG_START as ECU D forwards it: 12 bytes, little-endian fields. */
static void send_img_start(uint16_t w, uint16_t h, uint16_t chunks, uint32_t tick)
{
    uint8_t b[12];
    memset(b, 0, sizeof b);
    b[0] = (uint8_t)(w & 0xFF);      b[1] = (uint8_t)(w >> 8);
    b[2] = (uint8_t)(h & 0xFF);      b[3] = (uint8_t)(h >> 8);
    b[4] = (uint8_t)(chunks & 0xFF); b[5] = (uint8_t)(chunks >> 8);
    b[10] = 0x01;
    uint8_t chk = 0;
    for (int i = 0; i < 11; i++) chk ^= b[i];
    b[11] = chk;
    feed_can(IMG_CAN_ID_START, b, 12, tick);
}

/* One 0x301 data frame: [SEQ_HI][SEQ_LO][REAL_LEN][payload...] padded to 64. */
static void send_img_data(uint16_t seq, const uint8_t *payload, uint8_t len, uint32_t tick)
{
    uint8_t b[64];
    memset(b, 0, sizeof b);
    b[0] = (uint8_t)(seq >> 8);
    b[1] = (uint8_t)(seq & 0xFF);
    b[2] = len;
    memcpy(&b[3], payload, len);
    feed_can(IMG_CAN_ID_DATA, b, 64, tick);
}

/* Drive Update() so the ring buffer drains to the screen. */
static void drain(uint32_t tick)
{
    for (int i = 0; i < 4; i++) SCREENIMAGE_Update(tick + (uint32_t)i);
}

static void reset_state(void)
{
    fake_screen_reset();
    imgState          = IMG_IDLE;
    screenActive      = True;
    kfRingWriteIdx    = 0;
    kfRingReadIdx     = 0;
    kfDecodedBytes    = 0;
    kfFramesRx        = 0;
    kfRingDrops       = 0;
    kfStateDrops      = 0;
    tileAssemblyPos   = 0;
    tileReady         = False;
    fake_reset();
    /* The activity timeout reads xTaskGetTickCount(), so the fake clock is part
       of this module's state -- a case that leaves it advanced silently expires
       the hold in every case after it. */
    fake_tick = 0;
}

/* IMG_END as ECU D forwards it: 8 bytes, byte 4 = status. */
static void send_img_end(uint8_t status, uint32_t tick)
{
    uint8_t b[8];
    memset(b, 0, sizeof b);
    b[4] = status;
    FDCAN_RxHeaderTypeDef h;
    memset(&h, 0, sizeof(h));
    h.Identifier = IMG_CAN_ID_END;
    h.DataLength = FDCAN_DLC_BYTES_8;
    SCREENIMAGE_ProcessRxCANMessage(&h, b, tick);
}

/* The single 0x303 frame ECU A sends in reply to IMG_END, or NULL. */
static const CapturedFrame_t *last_ack(void)
{
    for (int i = fake_can_tx_count - 1; i >= 0; i--)
        if (fake_can_tx[i].header.Identifier == IMG_CAN_ID_ACK) return &fake_can_tx[i];
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Cases                                                               */
/* ------------------------------------------------------------------ */

static void case_rle_decode_matches_the_golden_vectors(void)
{
    h_case_begin("RLE_Decode reproduces every golden vector");
    /* The pure function, against an encoder written from the spec in another
       language. This is the half of the contract ECU A owns. */
    for (size_t i = 0; i < RAMN_RLE_VECTOR_COUNT; i++) {
        const ramn_rle_vector_t *v = &ramn_rle_vectors[i];
        uint8_t out[512];
        uint16_t n = RLE_Decode(v->encoded, (uint16_t)v->encoded_len, out, sizeof out);

        if (!CHECK_OK(n == v->raw_len, v->name)) continue;
        if (v->raw_len > 0) CHECK(memcmp(out, v->raw, v->raw_len) == 0, v->name);
    }
}

static void case_real_len_not_dlc_decides_the_payload(void)
{
    h_case_begin("REAL_LEN, not DLC, decides how much of a frame is real");
    /* The 1 September truncation. DLC is a fixed 64 on every data frame, so a
       receiver deriving the length from it appends the sender's zero padding
       to the stream as pixel data. */
    reset_state();
    send_img_start(240, 240, 1, 100);

    /* Three real bytes: a run of two pixels. Encoded: 0x81 then the pixel. */
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, 3, 101);
    drain(102);

    CHECK(fake_screen_len == 4, "two pixels reach the panel, not 61 bytes of padding");
    if (fake_screen_len >= 4) {
        const uint8_t want[4] = {0xAB, 0xCD, 0xAB, 0xCD};
        CHECK(memcmp(fake_screen, want, 4) == 0, "and they are the right pixels");
    }
}

static void case_data_before_start_is_dropped(void)
{
    h_case_begin("IMG_DATA before IMG_START is dropped");
    reset_state();
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, 3, 100);
    drain(101);
    CHECK(fake_screen_len == 0, "nothing reaches the panel outside a keyframe");
}

static void case_real_len_past_the_frame_is_clamped(void)
{
    h_case_begin("a REAL_LEN past the end of the frame is clamped");
    /* REAL_LEN crosses the bus unchecksummed by this layer. */
    reset_state();
    send_img_start(240, 240, 1, 100);

    uint8_t b[64];
    memset(b, 0, sizeof b);
    b[0] = 0; b[1] = 0;
    b[2] = 200;              /* impossible: only 61 payload bytes exist */
    b[3] = 0x81; b[4] = 0xAB; b[5] = 0xCD;
    feed_can(IMG_CAN_ID_DATA, b, 64, 101);
    drain(102);

    CHECK(fake_screen_len <= 61 * 2, "no read past the frame reaches the panel");
}

static void case_a_split_block_across_frames(void)
{
    h_case_begin("an RLE block split across two frames");
    /* The encoder RLEs the whole image as ONE stream and cuts it at fixed
       offsets, so a block can begin in one frame and end in the next. This
       decoder decodes each frame independently, so it cannot rejoin them.

       RLE_DecodeStream carries the half-read block across the boundary, so
       the control byte in frame 0 and its pixel bytes in frame 1 are one
       run. */
    reset_state();
    send_img_start(240, 240, 2, 100);

    /* Run of two pixels 0xABCD, with the control byte alone in frame 0 and
       its pixel bytes at the head of frame 1. */
    const uint8_t first[1]  = {0x81};
    const uint8_t second[2] = {0xAB, 0xCD};
    send_img_data(0, first, 1, 101);
    send_img_data(1, second, 2, 102);
    drain(103);

    CHECK(fake_screen_len == 4, "the split run still produces two pixels");
    if (fake_screen_len >= 4) {
        const uint8_t want[4] = {0xAB, 0xCD, 0xAB, 0xCD};
        CHECK(memcmp(fake_screen, want, 4) == 0, "and they are the right pixels");
    }
}

static void case_a_whole_keyframe(void)
{
    h_case_begin("a whole keyframe reaches the panel");
    /* End to end at real scale, using the chunk size the protocol defines. A
       correct receiver puts exactly one screen of pixels on the panel. */
    reset_state();

    static uint8_t raw[240 * 240 * 2];
    for (size_t i = 0; i < sizeof raw; i += 2) {      /* a compressible pattern */
        raw[i]     = (uint8_t)((i / 2 / 240) & 0xFF);
        raw[i + 1] = 0x20;
    }

    /* Encode it the way the ESP32 does: one stream over the whole frame. */
    static uint8_t stream[240 * 240 * 3];
    size_t sp = 0, rp = 0;
    while (rp + 1 < sizeof raw) {
        size_t run = 1;
        while (rp + 2 * (run + 1) <= sizeof raw && run < 128 &&
               raw[rp + 2 * run] == raw[rp] && raw[rp + 2 * run + 1] == raw[rp + 1]) run++;
        if (run >= 2) {
            stream[sp++] = (uint8_t)(0x80 | (run - 1));
            stream[sp++] = raw[rp]; stream[sp++] = raw[rp + 1];
            rp += 2 * run;
        } else {
            stream[sp++] = 1;                          /* two literal bytes */
            stream[sp++] = raw[rp]; stream[sp++] = raw[rp + 1];
            rp += 2;
        }
    }

    send_img_start(240, 240, (uint16_t)((sp + RAMN_PIPE_SPI_CHUNK_PAYLOAD - 1) /
                                        RAMN_PIPE_SPI_CHUNK_PAYLOAD), 200);
    uint16_t seq = 0;
    for (size_t off = 0; off < sp; off += RAMN_PIPE_SPI_CHUNK_PAYLOAD) {
        size_t n = sp - off;
        if (n > RAMN_PIPE_SPI_CHUNK_PAYLOAD) n = RAMN_PIPE_SPI_CHUNK_PAYLOAD;
        send_img_data(seq++, &stream[off], (uint8_t)n, 201);
        drain(202);              /* drain as we go: the ring holds few entries */
    }
    drain(203);

    CHECK(RLE_StreamMidBlock(&kfStream) == False,
          "the stream ends on a block boundary, with nothing half-read");

    CHECK(fake_screen_len == sizeof raw, "one full screen of pixels reaches the panel");
    CHECK(memcmp(fake_screen, raw, sizeof raw) == 0, "and every pixel is the right one");
}

static void case_odd_length_decode_is_not_lost(void)
{
    h_case_begin("a chunk decoding to an odd length still reaches the panel");
    /* Straight from a real keyframe: the stream opens with a 1-byte literal,
       so decoded lengths pass through odd values. RAMN_SPI_WriteImageChunk
       DISCARDS an odd-length write outright -- byte stream to us, pixels to
       the ST7789 -- so handing it one loses the whole chunk in silence. This
       is what a blank screen looked like on hardware.

       payload: 00 00   literal, one byte     -> 1 byte  (odd)
                84 FFFF run of 5 pixels       -> 10      (11, odd)
                81 0000 run of 2 pixels       -> 4       (15, odd) */
    reset_state();
    send_img_start(240, 240, 2, 100);

    const uint8_t first[]  = {0x00, 0x00, 0x84, 0xFF, 0xFF, 0x81, 0x00, 0x00};
    const uint8_t second[] = {0x00, 0x11};      /* a literal byte to pair the tail */
    send_img_data(0, first, sizeof first, 101);
    drain(102);
    send_img_data(1, second, sizeof second, 103);
    drain(104);

    /* 1 + 10 + 4 = 15 from the first chunk, then 1 more: 16 bytes, all of it. */
    CHECK(fake_screen_odd_drops == 0, "nothing was dropped for being odd-length");
    CHECK(fake_screen_len == 16, "every decoded byte reached the panel");
    if (fake_screen_len >= 15) {
        const uint8_t want[15] = {0x00,
                                  0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
                                  0x00,0x00, 0x00,0x00};
        CHECK(memcmp(fake_screen, want, 15) == 0, "and in the right order");
    }
}


static void case_img_start_is_acknowledged_on_the_bus(void)
{
    h_case_begin("IMG_START is acknowledged, not just IMG_END");
    /* ECU A is silent by construction. Without a START ack, an ECU A that
       receives 0x300 and never sees IMG_END looks exactly like one receiving
       nothing at all -- which is the state this hardware was actually in:
       no 0x303 of any kind ever reached ECU D. */
    reset_state();
    send_img_start(240, 240, 1, 100);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL, "IMG_START alone produces a 0x303")) return;
    CHECK(ack->len == 8, "carrying the 8-byte diagnostic payload");
    CHECK(ack->data[0] == IMG_ACK_START, "tagged as the START stage, not a completion");
    CHECK(ack->header.FDFormat == FDCAN_CLASSIC_CAN,
          "classic CAN, so a non-FD listener on the bus can still see it");

    /* And the END ack must still be distinguishable from it. */
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, sizeof payload, 101);
    drain(102);
    send_img_end(0x00, 103);

    const CapturedFrame_t *endAck = last_ack();
    if (!CHECK_OK(endAck != NULL, "IMG_END produces its own 0x303")) return;
    CHECK(endAck->data[0] != IMG_ACK_START, "and it is not tagged as a START");
    CHECK(endAck->data[0] == 0x00, "a clean keyframe reports status OK");
}

static void case_img_end_is_always_answered(void)
{
    h_case_begin("IMG_END is answered even when no keyframe is open");
    /* On hardware ECU D reached KEYFRAME_SENT -- so it DID put 0x302 on the bus
       -- and ECU A never replied. Silence there has two completely different
       causes with the same appearance: the frame never arrived, or it arrived
       after the keyframe had been torn down. Answering unconditionally splits
       them. */
    reset_state();
    imgState = IMG_IDLE;               /* no keyframe open */
    send_img_end(0x00, 100);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL, "an out-of-state IMG_END still produces a 0x303")) return;
    CHECK(ack->data[0] == IMG_ACK_LATE, "tagged 0x03: arrived, but nothing was open");
}

static void case_start_ack_carries_the_previous_frame(void)
{
    h_case_begin("the START ack reports the keyframe that just ended");
    /* The END ack is the one that goes missing, so it cannot be the only place
       these numbers live. At IMG_START the current counters are all zero by
       definition -- reporting them would say nothing. */
    reset_state();
    send_img_start(240, 240, 1, 100);
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, sizeof payload, 101);
    drain(102);
    size_t firstFrameBytes = fake_screen_len;

    fake_reset();                       /* forget the first frame's ACKs */
    send_img_start(240, 240, 1, 200);   /* second keyframe */

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "the second START is acked")) return;
    CHECK(ack->data[0] == IMG_ACK_START, "tagged as a START");
    uint32_t decoded = (uint32_t)ack->data[2]
                     | ((uint32_t)ack->data[3] << 8)
                     | ((uint32_t)ack->data[4] << 16);
    CHECK(decoded == firstFrameBytes, "and it reports the PREVIOUS frame's decoded bytes");
    CHECK(ack->data[5] == 1, "and the previous frame's accepted chunk count");
}

static void case_ack_carries_the_can_rx_overrun_count(void)
{
    h_case_begin("the ACK carries ECU A's FDCAN RX overrun count");
    /* 23 back-to-back 64-byte FD frames arrive in about 1.5 ms. If the
       peripheral drops any, this module never sees them and every counter it
       owns reads clean -- so the overrun count has to come from the driver, or
       a lost burst is invisible. */
    reset_state();
    RAMN_FDCAN_Status.CANRxOverrunCnt = 9;
    send_img_start(240, 240, 1, 100);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "the START is acked")) return;
    CHECK(ack->data[7] == 9, "byte 7 reports the driver's overrun count");

    fake_reset();                                 /* also zeroes the fake driver counters */
    RAMN_FDCAN_Status.CANRxOverrunCnt = 100000;   /* must not wrap into a small number */
    send_img_start(240, 240, 1, 200);
    const CapturedFrame_t *big = last_ack();
    if (!CHECK_OK(big != NULL, "acked again")) return;
    CHECK(big->data[7] == 255, "and saturates rather than wrapping");
}

static void case_a_lagging_periodic_tick_does_not_dismiss(void)
{
    h_case_begin("a lagging periodic tick does not dismiss the screen");
    /* The periodic task's xLastWakeTime falls permanently behind real time
       whenever the loop overruns -- and writing a keyframe to the panel is
       ~33 ms inside a 10 ms period, so it always does. lastActivityTick is
       taken with xTaskGetTickCount() in the CAN RX task, so the subtraction
       underflowed to ~4.29 billion and the 5 s hold expired instantly.

       On hardware this tore the screen down mid-keyframe on EVERY frame:
       imgState fell back to IMG_IDLE, the remaining chunks were rejected as
       out-of-state, and IMG_END came back as st=3 with decoded=0. */
    reset_state();
    fake_tick = 10000;                       /* real time */
    send_img_start(240, 240, 1, 10000);      /* lastActivityTick = 10000 */

    SCREENIMAGE_Update(500);                 /* a badly lagging xLastWakeTime */

    CHECK(screenActive != False, "the screen is still held");
    CHECK(imgState == KEYFRAME_RX, "and the keyframe is still open");
    CHECK(RAMN_SCREENIMAGE_DisplayRequested != False, "and still requested");

    /* A real idle period must still dismiss it. */
    fake_tick = 10000 + IMAGE_HOLD_MS + 1;
    SCREENIMAGE_Update(600);
    CHECK(screenActive == False, "but a genuine idle timeout still dismisses");
}

static void case_ack_reports_what_ecua_saw(void)
{
    h_case_begin("the 0x303 ACK reports what ECU A actually decoded");
    /* ECU A has no UART -- ENABLE_UART is TARGET_ECUD only -- so this ACK is
       the ONLY thing it can say about a keyframe. A blank screen and a stream
       that never arrived are indistinguishable without it. */
    reset_state();
    send_img_start(240, 240, 1, 100);

    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};   /* a run of two pixels */
    send_img_data(0, payload, 3, 101);
    drain(102);
    send_img_end(0x00, 103);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL, "IMG_END is answered with a 0x303 ACK")) return;

    CHECK(ack->len == 8, "the ACK carries the full 8-byte diagnostic payload");
    if (ack->len < 8) return;

    CHECK(ack->data[0] == 0x00, "status is OK on a clean stream");
    CHECK(ack->data[1] == 0x00, "no flags set: not truncated, nothing dropped");

    uint32_t decoded = (uint32_t)ack->data[2]
                     | ((uint32_t)ack->data[3] << 8)
                     | ((uint32_t)ack->data[4] << 16);
    CHECK(decoded == fake_screen_len, "the reported byte count is what reached the panel");
    CHECK(ack->data[5] == 1, "one data frame was accepted");
    CHECK(ack->data[6] == 0, "no ring-full drops");
    CHECK(ack->data[7] == 0, "no wrong-state drops");
}

static void case_ack_counts_a_full_keyframe(void)
{
    h_case_begin("the ACK's byte count survives a whole keyframe");
    /* A 240x240 RGB565 frame is 115,200 bytes. The counter behind this field
       was a uint16_t, so it wrapped at 65,536 and could never report a
       complete frame -- the one number worth reporting was the one it could
       not hold. */
    reset_state();

    /* A solid frame: one run block per 128 pixels, three source bytes each. */
    static uint8_t stream[240 * 240 / 128 * 3];
    for (size_t i = 0; i < sizeof stream; i += 3) {
        stream[i] = 0xFF;                    /* run of 128 */
        stream[i + 1] = 0x12; stream[i + 2] = 0x34;
    }

    uint16_t chunks = (uint16_t)((sizeof stream + RAMN_PIPE_SPI_CHUNK_PAYLOAD - 1) /
                                 RAMN_PIPE_SPI_CHUNK_PAYLOAD);
    send_img_start(240, 240, chunks, 200);
    uint16_t seq = 0;
    for (size_t off = 0; off < sizeof stream; off += RAMN_PIPE_SPI_CHUNK_PAYLOAD) {
        size_t n = sizeof stream - off;
        if (n > RAMN_PIPE_SPI_CHUNK_PAYLOAD) n = RAMN_PIPE_SPI_CHUNK_PAYLOAD;
        send_img_data(seq++, &stream[off], (uint8_t)n, 201);
        drain(202);
    }
    drain(203);
    send_img_end(0x00, 204);

    CHECK(fake_screen_len == 240 * 240 * 2, "a full screen of pixels reaches the panel");

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "an 8-byte ACK came back")) return;
    uint32_t decoded = (uint32_t)ack->data[2]
                     | ((uint32_t)ack->data[3] << 8)
                     | ((uint32_t)ack->data[4] << 16);
    CHECK(decoded == 240u * 240u * 2u, "and it reports all 115,200 bytes, not 115,200 mod 65,536");
}

static void case_the_ring_absorbs_a_whole_keyframe(void)
{
    h_case_begin("the ring absorbs a keyframe arriving faster than it drains");
    /* The producer and consumer are not rate-matched. ECU D polls the ESP32
       every 1 ms while streaming and each poll yields two chunks (~2 CAN
       frames/ms), while SCREENIMAGE_Update drains on the 10 ms periodic task
       and a full panel write is ~33 ms. So a whole keyframe lands before the
       first drain finishes, and the ring has to hold it.

       At 8 entries this dropped over half of every keyframe -- and because the
       RLE stream is continuous, a dropped chunk desynchronises every byte
       after it, so the screen showed nothing rather than a partial image. */
    reset_state();

    const uint16_t chunks = 45;   /* the upper end of what this canvas produces */
    send_img_start(240, 240, chunks, 500);

    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};   /* one run of two pixels */
    for (uint16_t i = 0; i < chunks; i++)
        send_img_data(i, payload, sizeof payload, 501);   /* no drain in between */

    CHECK(kfRingDrops == 0, "a whole keyframe arrives with nothing dropped");
    CHECK(kfFramesRx == chunks, "every chunk was accepted");

    drain(502);
    CHECK(fake_screen_len == (size_t)chunks * 4u,
          "and every chunk's pixels reach the panel once drained");
}

static void case_ack_reports_a_ring_overflow(void)
{
    h_case_begin("chunks dropped by a full ring are reported, not silent");
    /* The ring holds KFRING_ENTRIES-1 frames and is drained by the Periodic
       task. If that task is busy -- SCREENIMAGE_Init alone paints 115,200
       bytes of black over SPI -- frames arrive with nowhere to go. Dropping
       one desynchronises the RLE stream and corrupts every pixel after it, so
       a silent drop presents as a decoder fault. */
    reset_state();
    send_img_start(240, 240, 32, 300);

    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    for (int i = 0; i < KFRING_ENTRIES + 4; i++)        /* no drain in between */
        send_img_data((uint16_t)i, payload, sizeof payload, 301);

    CHECK(kfRingDrops > 0, "the ring did overflow with no drain running");
    drain(302);
    send_img_end(0x00, 303);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "an 8-byte ACK came back")) return;
    CHECK(ack->data[0] == 0x01, "status is not OK when chunks were lost");
    CHECK((ack->data[1] & 0x02) != 0, "the ring-drop flag is set");
    CHECK(ack->data[6] == (uint8_t)kfRingDrops, "and the drop count is reported");
}

static void case_ack_reports_wrong_state_drops(void)
{
    h_case_begin("chunks arriving outside KEYFRAME_RX are counted");
    /* Data after IMG_END, or with no IMG_START at all, is dropped by design.
       Counting it separates "ECU D sent nothing" from "ECU A threw it away". */
    reset_state();
    send_img_start(240, 240, 1, 400);
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, sizeof payload, 401);
    drain(402);
    send_img_end(0x00, 403);            /* -> IMG_SHOWN */

    const CapturedFrame_t *first = last_ack();
    if (!CHECK_OK(first != NULL, "the first IMG_END is acknowledged")) return;
    CHECK(first->data[7] == 0, "nothing was dropped for state before IMG_END");

    send_img_data(1, payload, sizeof payload, 404);   /* too late */
    CHECK(kfStateDrops == 1, "a late data frame is counted, not ignored");
}

int main(void)
{
    printf("ECU A image screen host tests\n");
    printf("(assertion surface: bytes handed to RAMN_SPI_WriteImageChunk)\n");

    case_rle_decode_matches_the_golden_vectors();
    case_real_len_not_dlc_decides_the_payload();
    case_data_before_start_is_dropped();
    case_real_len_past_the_frame_is_clamped();
    case_a_split_block_across_frames();
    case_a_whole_keyframe();
    case_odd_length_decode_is_not_lost();
    case_img_start_is_acknowledged_on_the_bus();
    case_img_end_is_always_answered();
    case_a_lagging_periodic_tick_does_not_dismiss();
    case_start_ack_carries_the_previous_frame();
    case_ack_carries_the_can_rx_overrun_count();
    case_ack_reports_what_ecua_saw();
    case_ack_counts_a_full_keyframe();
    case_the_ring_absorbs_a_whole_keyframe();
    case_ack_reports_a_ring_overflow();
    case_ack_reports_wrong_state_drops();

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
