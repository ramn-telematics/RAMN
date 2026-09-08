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
static void send_img_start_scaled(uint16_t w, uint16_t h, uint16_t chunks,
                                  uint8_t scale, uint32_t tick)
{
    uint8_t b[12];
    memset(b, 0, sizeof b);
    b[8] = scale;
    b[0] = (uint8_t)(w & 0xFF);      b[1] = (uint8_t)(w >> 8);
    b[2] = (uint8_t)(h & 0xFF);      b[3] = (uint8_t)(h >> 8);
    b[4] = (uint8_t)(chunks & 0xFF); b[5] = (uint8_t)(chunks >> 8);
    b[10] = 0x01;
    uint8_t chk = 0;
    for (int i = 0; i < 11; i++) chk ^= b[i];
    b[11] = chk;
    feed_can(IMG_CAN_ID_START, b, 12, tick);
}

/* Unscaled, which is every case that predates the scale byte. */
static void send_img_start(uint16_t w, uint16_t h, uint16_t chunks, uint32_t tick)
{
    send_img_start_scaled(w, h, chunks, 1, tick);
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
    kfTileDrops       = 0;
    kfTileShort       = 0;
    tileActive        = False;
    kfScale           = 1;
    rxScale           = 1;
    kfRowFill         = 0;
    memset(&prevStats, 0, sizeof prevStats);
    kfEndSeen         = False;
    kfExpectedSeq     = 0;
    kfSeqBroken       = False;
    kfDupSkips        = 0;
    RLE_StreamReset(&tileStream);
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
    /* IMG_END rides the ring behind the chunks it terminates, so the ACK comes
       out of the periodic task rather than the CAN task. Every caller here
       wants the answer, so drain for it. */
    drain(tick);
}

/* The single 0x303 frame ECU A sends in reply to IMG_END, or NULL. */
static const CapturedFrame_t *last_ack(void)
{
    for (int i = fake_can_tx_count - 1; i >= 0; i--)
        if (fake_can_tx[i].header.Identifier == IMG_CAN_ID_ACK) return &fake_can_tx[i];
    return NULL;
}


/* ------------------------------------------------------------------ */
/* Delta fixtures                                                       */
/* ------------------------------------------------------------------ */

/* Put ECU A into the state a delta frame requires: a keyframe has completed,
   so imgState is IMG_SHOWN. */
static void reach_img_shown(void)
{
    reset_state();
    send_img_start(240, 240, 1, 100);
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, sizeof payload, 101);
    drain(102);
    send_img_end(0x00, 103);
    fake_screen_reset();
    fake_reset();
}

/* 0x305 as ECU D forwards it: [X][Y][SIZE][SEQ][LEN][RLE...] padded to 64. */
static void send_tile_chunk(uint8_t tx, uint8_t ty, uint8_t size, uint8_t seq,
                            const uint8_t *rle, uint8_t len, uint32_t tick)
{
    uint8_t b[64];
    memset(b, 0, sizeof b);
    b[0] = tx; b[1] = ty; b[2] = size; b[3] = seq; b[4] = len;
    memcpy(&b[5], rle, len);
    FDCAN_RxHeaderTypeDef h;
    memset(&h, 0, sizeof(h));
    h.Identifier = DELTA_CAN_ID_TILE_CHUNK;
    h.DataLength = FDCAN_DLC_BYTES_64;
    SCREENIMAGE_ProcessRxCANMessage(&h, b, tick);
}

/* RLE-encode a run of identical pixels the way the sender does. */
static uint16_t rle_solid(uint8_t *out, uint16_t pixels, uint8_t hi, uint8_t lo)
{
    uint16_t n = 0;
    while (pixels) {
        uint16_t run = pixels > 128 ? 128 : pixels;
        out[n++] = (uint8_t)(0x80 | (run - 1));
        out[n++] = hi; out[n++] = lo;
        pixels = (uint16_t)(pixels - run);
    }
    return n;
}

/* Cut an RLE stream into 59-byte chunks and send them as one tile, with bit 7
   of the sequence set on the last -- exactly ECU D's framing. */
static void send_tile(uint8_t tx, uint8_t ty, uint8_t size,
                      const uint8_t *rle, uint16_t rleLen, uint32_t tick)
{
    const uint8_t CH = 59;
    uint16_t nchunks = (uint16_t)((rleLen + CH - 1) / CH);
    for (uint16_t i = 0; i < nchunks; i++) {
        uint16_t off = (uint16_t)(i * CH);
        uint8_t n = (uint8_t)((rleLen - off) > CH ? CH : (rleLen - off));
        uint8_t seq = (uint8_t)(i & 0x7F);
        if (i == nchunks - 1) seq |= 0x80;
        send_tile_chunk(tx, ty, size, seq, &rle[off], n, tick);
    }
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
    /* The START ack now rides the ring with the frame boundary, so it comes out
       of the periodic task -- the same reason send_img_end() drains. Sent where
       IMG_START arrives, it described a previous frame that still had chunks
       queued behind it. */
    drain(100);

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
    drain(200);

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
    drain(100);

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "the START is acked")) return;
    CHECK(ack->data[7] == 9, "byte 7 reports the driver's overrun count");

    fake_reset();                                 /* also zeroes the fake driver counters */
    RAMN_FDCAN_Status.CANRxOverrunCnt = 100000;   /* must not wrap into a small number */
    send_img_start(240, 240, 1, 200);
    drain(200);
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


static void case_a_delta_tile_reaches_the_panel(void)
{
    h_case_begin("a delta tile is decoded and written to the panel");
    reach_img_shown();

    /* An 8x8 tile of one colour: 64 pixels, 128 bytes. */
    uint8_t rle[64];
    uint16_t n = rle_solid(rle, 64, 0x12, 0x34);
    send_tile(3, 5, 8, rle, n, 200);
    drain(201);

    CHECK(kfTileDrops == 0, "the tile passes validation");
    CHECK(fake_window_opens == 1, "one window is opened for it");
    CHECK(fake_window_w == 8 && fake_window_h == 8, "sized to the tile");
    CHECK(fake_screen_len == 8 * 8 * 2, "and a whole tile of pixels is written");
    if (fake_screen_len >= 2)
        CHECK(fake_screen[0] == 0x12 && fake_screen[1] == 0x34, "with the right pixel");
}

static void case_a_tile_split_across_chunks(void)
{
    h_case_begin("a tile whose RLE spans many chunks still decodes");
    /* THE defect this rework exists for. A 40x40 tile is 3,200 bytes and ECU D
       caps a chunk at 59, so the stream is cut into ~26 pieces with blocks
       straddling the cuts. Decoding each chunk on its own loses every straddling
       block -- which was most of them. */
    reach_img_shown();

    static uint8_t rle[4096];
    uint16_t n = 0;
    /* Alternating short runs, so blocks land across chunk boundaries. */
    for (uint16_t px = 0; px < 40 * 40; ) {
        uint16_t run = (px / 7) % 5 + 1;
        if (px + run > 40 * 40) run = (uint16_t)(40 * 40 - px);
        rle[n++] = (uint8_t)(0x80 | (run - 1));
        rle[n++] = (uint8_t)(px & 0xFF);
        rle[n++] = (uint8_t)(px >> 8);
        px = (uint16_t)(px + run);
    }
    CHECK(n > 59 * 4, "the fixture really does span many chunks");

    /* Reference: the whole stream decoded in one go. */
    static uint8_t want[TILE_RAW_MAX];
    {
        RleStream_t st; RLE_StreamReset(&st);
        uint16_t got = RLE_DecodeStream(&st, rle, n, want, sizeof want);
        CHECK(got >= 40 * 40 * 2, "reference decode produces a full tile");
    }

    /* Show the old approach really does get it wrong on this fixture, so this
       case demonstrably tests the fix rather than merely passing. Decoding each
       59-byte chunk on its own is what the module used to do.

       Compared by CONTENT, not by length: a misparse happily over-produces and
       is then clipped at the buffer, so the byte count comes out right while
       the pixels are garbage. Checking the count alone passed here, which is
       exactly the kind of weak assertion this case exists to avoid. */
    {
        static uint8_t naive[TILE_RAW_MAX];
        memset(naive, 0, sizeof naive);
        uint16_t pos = 0;
        for (uint16_t off = 0; off < n && pos < TILE_RAW_MAX; off += 59) {
            uint16_t len = (uint16_t)((n - off) > 59 ? 59 : (n - off));
            pos = (uint16_t)(pos + RLE_Decode(&rle[off], len, &naive[pos],
                                              (uint16_t)(TILE_RAW_MAX - pos)));
        }
        CHECK(memcmp(naive, want, 40 * 40 * 2) != 0,
              "per-chunk decoding produces the wrong pixels -- the old bug is real");
    }

    send_tile(0, 0, 40, rle, n, 300);
    drain(301);

    CHECK(kfTileShort == 0, "the tile is not reported short");
    CHECK(fake_screen_len == 40 * 40 * 2, "every pixel of the tile reaches the panel");
    CHECK(memcmp(fake_screen, want, 40 * 40 * 2) == 0, "and every pixel is the right one");
}

static void case_many_tiles_in_one_frame(void)
{
    h_case_begin("a whole delta frame of tiles arrives without dropping any");
    /* The old path staged ONE finished tile and dropped any that arrived while
       it waited for the 10 ms periodic task. A delta frame is many tiles back
       to back, so most of it disappeared. */
    reach_img_shown();

    const int TILES = 12;
    uint8_t rle[64];
    uint16_t n = rle_solid(rle, 64, 0xAB, 0xCD);
    for (int i = 0; i < TILES; i++)
        send_tile((uint8_t)i, (uint8_t)i, 8, rle, n, 400);   /* no drain between */
    drain(401);

    CHECK(kfRingDrops == 0, "none were dropped for want of a slot");
    CHECK(fake_window_opens == TILES, "each tile opened its own window");
    CHECK(fake_screen_len == (size_t)TILES * 8 * 8 * 2, "and every tile was written");
}

static void case_a_tile_that_would_overhang_is_refused(void)
{
    h_case_begin("a tile that would hang off the panel is refused, not clamped");
    /* Tile coordinates are on an 8-pixel grid, tileSize is the extent, so a
       40-wide tile at tileX=29 starts at x=232 and needs 32 pixels that do not
       exist. Clamping the window and writing the tile's bytes into it shifts
       every row -- corrupt, and drawn with confidence. */
    reach_img_shown();

    uint8_t rle[4096];
    uint16_t n = rle_solid(rle, 40 * 40, 0x11, 0x22);
    send_tile(29, 0, 40, rle, n, 500);
    drain(501);

    CHECK(kfTileDrops > 0, "the overhanging tile is counted as refused");
    CHECK(fake_window_opens == 0, "no window is opened for it");
    CHECK(fake_screen_len == 0, "and nothing is written to the panel");

    /* The largest tile that does fit at that size must still be accepted. */
    fake_screen_reset();
    send_tile(25, 25, 40, rle, n, 502);      /* 200..239 -- exactly flush */
    drain(503);
    CHECK(fake_screen_len == 40 * 40 * 2, "a tile flush with the edge is accepted");
}

static void case_a_bad_tile_size_is_refused(void)
{
    h_case_begin("an impossible tile size is refused");
    reach_img_shown();
    uint8_t rle[8];
    uint16_t n = rle_solid(rle, 1, 0x00, 0x00);
    send_tile_chunk(0, 0, 24, 0x80, rle, (uint8_t)n, 600);   /* 24 is not 8/16/40 */
    drain(601);
    CHECK(kfTileDrops == 1, "the size is rejected");
    CHECK(fake_screen_len == 0, "and nothing reaches the panel");
}

static void case_a_truncated_tile_is_not_drawn(void)
{
    h_case_begin("a tile whose chunks stop early is counted, not half-drawn");
    reach_img_shown();

    /* Claim the last chunk while only part of the tile has been sent. */
    uint8_t rle[64];
    uint16_t n = rle_solid(rle, 8, 0x77, 0x88);      /* 8 pixels of a 64-pixel tile */
    send_tile(1, 1, 8, rle, n, 700);
    drain(701);

    CHECK(kfTileShort == 1, "the short tile is counted");
    CHECK(fake_screen_len == 0, "and no partial tile is written");
}

static void case_a_tile_missing_its_first_chunk_is_refused(void)
{
    h_case_begin("a tile whose first chunk was lost is refused, not misplaced");
    /* Every chunk carries its own coordinates, but geometry is latched only on
       chunk 0. If chunk 0 is dropped -- ring full, or a lost CAN frame -- the
       remaining chunks would decode into whatever tile was current and paint at
       the WRONG place, confidently. Nothing drops on the host, so only a lossy
       bus produces this; it has to be refused by construction. */
    reach_img_shown();

    /* One good tile at (1,1), so there is a previous geometry to inherit. */
    uint8_t rle[64];
    uint16_t n = rle_solid(rle, 64, 0x11, 0x11);
    send_tile(1, 1, 8, rle, n, 900);
    drain(901);
    size_t after_good = fake_screen_len;
    CHECK(after_good == 8 * 8 * 2, "the good tile is drawn");

    /* Now a DIFFERENT tile at (7,9) whose first chunk never arrives. */
    uint16_t before = kfTileDrops;
    send_tile_chunk(7, 9, 8, 0x81, rle, (uint8_t)n, 902);   /* seq 1, and last */
    drain(903);

    CHECK(kfTileDrops == before + 1, "the orphaned chunk is refused");
    CHECK(fake_screen_len == after_good, "and nothing more is painted");

    /* The panel model proves it did not land on the earlier tile either. */
    size_t painted = 0;
    for (int y = 0; y < 240; y++)
        for (int x = 0; x < 240; x++) {
            size_t o = ((size_t)y * 240 + x) * 2;
            if (fake_panel[o] || fake_panel[o + 1]) painted++;
        }
    CHECK(painted == 8 * 8, "exactly the one good tile is on the panel");
}

static void case_tiles_before_a_keyframe_are_dropped(void)
{
    h_case_begin("delta tiles before any keyframe are dropped");
    reset_state();
    imgState = IMG_IDLE;
    uint8_t rle[64];
    uint16_t n = rle_solid(rle, 64, 0x01, 0x02);
    send_tile(0, 0, 8, rle, n, 800);
    drain(801);
    CHECK(kfStateDrops > 0, "they are counted as out-of-state");
    CHECK(fake_screen_len == 0, "and nothing is drawn");
}

static void case_img_end_does_not_overtake_the_ring(void)
{
    h_case_begin("IMG_END waits for the chunks that came before it");
    /* IMG_END is handled in the ReceiveCAN task; chunks are decoded by the
       10 ms Periodic task draining kfRingBuf. Nothing orders the two, so a
       keyframe whose chunks are still queued is measured -- and acknowledged
       -- as truncated.

       This stayed hidden while ECU D forwarded two chunks per SPI transaction:
       the ring drained between bursts. Raising the transaction to 512 bytes
       (7 chunks) put a whole burst plus IMG_END on the bus inside one 10 ms
       tick, and every keyframe since ACKs truncated. Hardware, 2026-09-08:
       23 chunks accepted, drop=0/0, decoded=51968/115200. 51968 bytes is
       exactly the 10 chunks that had been drained when IMG_END landed.

       Every other case here calls drain() before send_img_end(), so the whole
       suite encoded the assumption the hardware just broke. */
    reset_state();

    static uint8_t stream[240 * 240 / 128 * 3];
    for (size_t i = 0; i < sizeof stream; i += 3) {
        stream[i] = 0xFF;                    /* run of 128 */
        stream[i + 1] = 0x12; stream[i + 2] = 0x34;
    }

    uint16_t chunks = (uint16_t)((sizeof stream + RAMN_PIPE_SPI_CHUNK_PAYLOAD - 1) /
                                 RAMN_PIPE_SPI_CHUNK_PAYLOAD);
    send_img_start(240, 240, chunks, 200);

    /* The burst as the wire delivers it: no periodic tick in between. */
    uint16_t seq = 0;
    for (size_t off = 0; off < sizeof stream; off += RAMN_PIPE_SPI_CHUNK_PAYLOAD) {
        size_t n = sizeof stream - off;
        if (n > RAMN_PIPE_SPI_CHUNK_PAYLOAD) n = RAMN_PIPE_SPI_CHUNK_PAYLOAD;
        send_img_data(seq++, &stream[off], (uint8_t)n, 201);
    }
    send_img_end(0x00, 201);
    drain(202);

    CHECK(fake_screen_len == 240 * 240 * 2, "the whole frame still reaches the panel");

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "an 8-byte ACK came back")) return;
    uint32_t decoded = (uint32_t)ack->data[2]
                     | ((uint32_t)ack->data[3] << 8)
                     | ((uint32_t)ack->data[4] << 16);
    CHECK(decoded == 240u * 240u * 2u,
          "and the ACK reports the whole frame, not what happened to be drained");
    CHECK((ack->data[1] & 0x01U) == 0U, "with the truncated flag clear");
}

static void case_a_repeated_chunk_is_refused(void)
{
    h_case_begin("a chunk delivered twice is not decoded twice");
    /* The RLE stream is one continuous byte sequence cut at fixed offsets, so
       a chunk appended twice shifts every block after it -- the same damage as
       a dropped chunk. SEQ_HI/SEQ_LO are read off the frame and then ignored.

       The ESP32 decides whether ECU D took the staged MISO bytes by looking at
       MOSI, and re-presents the stage when the master's write did not parse as
       a poll. On a full-duplex bus ECU D has already clocked those bytes out,
       so it forwards them a second time. Hardware, 2026-09-08: IMG_START
       announced 1388 chunks, ECU D forwarded 1542, and ECU A decoded
       127,900 bytes against a 115,200-byte frame -- the 154 duplicates at 61
       bytes each, times this stream's 1.36x expansion, is 12,770 against the
       12,700 of overshoot measured. */
    reset_state();

    send_img_start(240, 240, 4, 300);
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};   /* one run of two pixels */
    send_img_data(0, payload, sizeof payload, 301);
    send_img_data(1, payload, sizeof payload, 301);
    send_img_data(1, payload, sizeof payload, 301);   /* the same chunk again */
    send_img_data(2, payload, sizeof payload, 301);
    drain(302);

    CHECK(fake_screen_len == 3u * 4u,
          "three distinct chunks paint three chunks' worth of pixels");
    CHECK(kfDecodedBytes == 3u * 4u, "and the byte count matches what was sent once");
}

static void case_a_chunk_arriving_out_of_order_is_refused(void)
{
    h_case_begin("a gap in the chunk sequence is caught, not decoded");
    /* A missing chunk desynchronises the stream exactly as a duplicate does.
       Splicing the next chunk in over the gap produces a plausible-looking
       image made of the wrong bytes, so the gap has to end the frame. */
    reset_state();

    send_img_start(240, 240, 4, 400);
    const uint8_t payload[3] = {0x81, 0xAB, 0xCD};
    send_img_data(0, payload, sizeof payload, 401);
    send_img_data(2, payload, sizeof payload, 401);   /* chunk 1 never arrived */
    drain(402);
    send_img_end(0x00, 403);

    CHECK(fake_screen_len == 1u * 4u, "only the chunk before the gap is drawn");

    const CapturedFrame_t *ack = last_ack();
    if (!CHECK_OK(ack != NULL && ack->len == 8, "an 8-byte ACK came back")) return;
    CHECK((ack->data[1] & 0x01U) != 0U, "and the ACK says the frame was truncated");
}

/* ------------------------------------------------------------------ */
/* Scaled keyframes                                                     */
/* ------------------------------------------------------------------ */

/* A valid stream for arbitrary pixels: literal blocks of up to 128 bytes.
   128 is not a multiple of a 60-pixel row, so blocks straddle row boundaries
   throughout -- which is the case the row assembler has to get right. */
static uint16_t rle_literal(uint8_t *out, const uint8_t *px, uint16_t nbytes)
{
    uint16_t o = 0, i = 0;
    while (i < nbytes) {
        uint16_t n = (uint16_t)(nbytes - i);
        if (n > 128) n = 128;
        out[o++] = (uint8_t)(n - 1);
        memcpy(&out[o], &px[i], n);
        o = (uint16_t)(o + n);
        i = (uint16_t)(i + n);
    }
    return o;
}

/* Distinct-ish pixels, so a misplaced one shows up rather than blending in. */
static void make_source(uint8_t *px, uint16_t w, uint16_t h)
{
    for (uint16_t y = 0; y < h; y++)
        for (uint16_t x = 0; x < w; x++) {
            uint16_t v = (uint16_t)(((x * 7u + y * 31u) & 0xFFFFu) | 1u);
            px[(y * w + x) * 2]     = (uint8_t)(v & 0xFF);
            px[(y * w + x) * 2 + 1] = (uint8_t)(v >> 8);
        }
}

static void send_scaled_frame(uint16_t w, uint16_t h, uint8_t scale,
                              const uint8_t *px, uint32_t tick)
{
    static uint8_t stream[80000];
    uint16_t n = rle_literal(stream, px, (uint16_t)(w * h * 2));
    uint16_t chunks = (uint16_t)((n + RAMN_PIPE_SPI_CHUNK_PAYLOAD - 1) /
                                 RAMN_PIPE_SPI_CHUNK_PAYLOAD);
    send_img_start_scaled(w, h, chunks, scale, tick);
    uint16_t seq = 0;
    for (uint16_t off = 0; off < n; off = (uint16_t)(off + RAMN_PIPE_SPI_CHUNK_PAYLOAD)) {
        uint16_t take = (uint16_t)(n - off);
        if (take > RAMN_PIPE_SPI_CHUNK_PAYLOAD) take = RAMN_PIPE_SPI_CHUNK_PAYLOAD;
        send_img_data(seq++, &stream[off], (uint8_t)take, tick + 1);
        drain(tick + 2);            /* the ring holds 63, this frame is ~120 */
    }
    drain(tick + 3);
}

static void case_a_scaled_keyframe_fills_the_panel(void)
{
    h_case_begin("a 60x60 frame at scale 4 fills the whole panel");
    /* Flattening in the browser paints f x f blocks flat but still ships all
       57,600 pixels and leaves the RLE to find the runs, which it only partly
       does. Sending the small image instead and repeating pixels here is the
       same picture for a fraction of the link: measured on a photographic
       frame, 3x3 flattened at full size is 824 chunks against 206 for 80x80
       scaled. */
    reset_state();
    static uint8_t src[60 * 60 * 2];
    make_source(src, 60, 60);
    send_scaled_frame(60, 60, 4, src, 200);

    CHECK(fake_window_opens == 1, "one window is opened");
    CHECK(fake_window_w == 240 && fake_window_h == 240,
          "sized to the panel, not to the 60x60 source");
    CHECK(fake_panel_oob == 0, "and nothing is written outside it");

    size_t wrong = 0, checked = 0;
    for (uint16_t y = 0; y < 240 && wrong == 0; y++)
        for (uint16_t x = 0; x < 240; x++) {
            size_t d = ((size_t)y * 240 + x) * 2;
            size_t s = ((size_t)(y / 4) * 60 + (x / 4)) * 2;
            checked++;
            if (fake_panel[d] != src[s] || fake_panel[d + 1] != src[s + 1]) { wrong++; break; }
        }
    CHECK(checked == 240u * 240u, "every panel pixel was compared");
    CHECK(wrong == 0, "and each one carries the source pixel it magnifies");
}

static void case_a_scale_that_would_overflow_is_refused(void)
{
    h_case_begin("a scale that would not fit the panel falls back to 1:1");
    /* A wrong scale does not crop the picture, it shears every row after the
       first. A 1:1 image in the corner is at least recognisably wrong. */
    reset_state();
    static uint8_t src[120 * 120 * 2];
    make_source(src, 120, 120);
    send_scaled_frame(120, 120, 3, src, 300);      /* 360 px would not fit */

    CHECK(fake_window_w == 120 && fake_window_h == 120,
          "the window is the source size, not 360x360");
    CHECK(fake_panel_oob == 0, "and nothing is written off the panel");
}

static void case_scale_zero_means_one_to_one(void)
{
    h_case_begin("a sender with no scale byte still draws 1:1");
    /* Byte 8 was padding before this existed, so an older ECU D sends 0. */
    reset_state();
    static uint8_t src[64 * 2];
    make_source(src, 64, 1);
    send_scaled_frame(64, 1, 0, src, 400);

    CHECK(fake_window_w == 64 && fake_window_h == 1, "the window is 64x1");
    CHECK(memcmp(fake_panel, src, 64 * 2) == 0, "and the pixels land unscaled");
}

static void case_tiles_are_refused_while_scaled(void)
{
    h_case_begin("delta tiles are refused against a scaled keyframe");
    /* Tiles carry source-space coordinates and no scale of their own, so
       against a scaled keyframe they would land at a fraction of their real
       position and size. The sender holds scale at 1 in delta mode; if it
       does not, refusing keeps the keyframe intact. */
    reset_state();
    static uint8_t src[60 * 60 * 2];
    make_source(src, 60, 60);
    send_scaled_frame(60, 60, 4, src, 500);
    send_img_end(0x00, 600);                       /* -> IMG_SHOWN */
    fake_screen_reset();

    uint8_t rle[8];
    uint16_t n = rle_solid(rle, 64, 0x12, 0x34);
    send_tile(3, 5, 8, rle, n, 700);
    drain(701);

    CHECK(kfTileDrops == 1, "the tile is counted as dropped");
    CHECK(fake_screen_len == 0, "and nothing is drawn over the keyframe");
}

/* ------------------------------------------------------------------ */
/* Frame boundaries under load                                          */
/*                                                                      */
/* At 100 ms and a low resolution the pipeline finally keeps up, which  */
/* means IMG_START for frame N+1 can arrive while ECU A is still        */
/* painting frame N. Painting is the slow half: a full panel is 115,200 */
/* bytes at 32 MHz, ~29 ms, and it blocks the periodic task the whole   */
/* time. Every case above drains to quiescence between frames, so none  */
/* of them can see what happens when the frames overlap.                */
/* ------------------------------------------------------------------ */

/* A run-encoded solid colour: 3 bytes per 128 pixels, so a whole 60x60
   frame is 87 bytes -- two chunks, and the ring can hold several frames'
   worth undrained. Solid colours also make "whose pixel is this?" a
   question the panel model can answer. */
static uint16_t rle_runs(uint8_t *out, uint8_t lo, uint8_t hi, uint32_t pixels)
{
    uint16_t o = 0;
    while (pixels) {
        uint32_t n = (pixels > 128u) ? 128u : pixels;
        out[o++] = (uint8_t)(0x80u | (n - 1u));
        out[o++] = lo;
        out[o++] = hi;
        pixels -= n;
    }
    return o;
}

/* IMG_END without the drain send_img_end() does -- these cases need the
   ring left deliberately full. */
static void feed_img_end(uint8_t status, uint32_t tick)
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

/* Queue a whole solid 60x60 scale-4 frame WITHOUT draining it. */
static void queue_solid_frame(uint8_t lo, uint8_t hi, uint32_t tick)
{
    uint8_t stream[256];
    uint16_t n = rle_runs(stream, lo, hi, 60u * 60u);
    uint16_t chunks = (uint16_t)((n + RAMN_PIPE_SPI_CHUNK_PAYLOAD - 1) /
                                 RAMN_PIPE_SPI_CHUNK_PAYLOAD);
    send_img_start_scaled(60, 60, chunks, 4, tick);
    uint16_t seq = 0;
    for (uint16_t off = 0; off < n; off = (uint16_t)(off + RAMN_PIPE_SPI_CHUNK_PAYLOAD)) {
        uint16_t take = (uint16_t)(n - off);
        if (take > RAMN_PIPE_SPI_CHUNK_PAYLOAD) take = RAMN_PIPE_SPI_CHUNK_PAYLOAD;
        send_img_data(seq++, &stream[off], (uint8_t)take, tick);
    }
    feed_img_end(0x00, tick);
}

/* How many panel pixels carry this exact colour. */
static size_t panel_count(uint8_t lo, uint8_t hi)
{
    size_t n = 0;
    for (size_t i = 0; i < sizeof fake_panel; i += 2)
        if (fake_panel[i] == lo && fake_panel[i + 1] == hi) n++;
    return n;
}

static void case_a_new_keyframe_does_not_discard_the_one_still_queued(void)
{
    h_case_begin("a keyframe arriving while the previous one is still queued does not erase it");
    /* Frame A is queued whole and never drained -- the periodic task is busy
       painting, which is the normal state of affairs at 10 fps. Then frame B's
       IMG_START arrives. IMG_START runs on the CAN RX task and zeroes
       kfRingWriteIdx and kfRingReadIdx, so every entry frame A left in the
       ring, its IMG_END included, is thrown away by the producer. Frame A is
       not torn or partial: it never reaches the glass at all. */
    reset_state();
    queue_solid_frame(0x11, 0x22, 200);
    queue_solid_frame(0x33, 0x44, 300);
    drain(400);

    CHECK(fake_window_opens == 2, "each keyframe opens its own window");
    CHECK(fake_screen_len == 2u * 240u * 240u * 2u,
          "and both frames' pixels reach the panel, not just the last one");
}

/* Fires from inside RAMN_SPI_WriteImageChunk on the Nth write, standing in for
   the CAN RX task running while the periodic task is blocked on the DMA. */
static int      preempt_at;
static int      preempt_seen;
static uint32_t preempt_tick;
static void preempt_with_img_start(void)
{
    if (++preempt_seen != preempt_at) return;
    void (*save)(void) = fake_screen_on_write;
    fake_screen_on_write = NULL;          /* the injected frame must not recurse */
    queue_solid_frame(0x33, 0x44, preempt_tick);
    fake_screen_on_write = save;
}

static void case_a_keyframe_starting_mid_paint_does_not_corrupt_the_panel(void)
{
    h_case_begin("a keyframe starting while the panel is mid-write does not shear the picture");
    /* The window above is not a knife edge: the drain holds the CPU inside a
       blocking SPI write for ~29 ms out of every 100, so this is where roughly
       a third of frame boundaries land. Resetting the ring indices under a
       running drain leaves the drain's local read index pointing into a region
       the producer has begun refilling: it keeps consuming frame A's entries
       and decodes them with frame B's geometry, into frame B's window.
       Nothing counts it and nothing reports it -- the only symptom is on the
       glass. */
    reset_state();
    preempt_at   = 3;
    preempt_seen = 0;
    preempt_tick = 300;
    queue_solid_frame(0x11, 0x22, 200);
    fake_screen_on_write = preempt_with_img_start;
    drain(400);
    fake_screen_on_write = NULL;
    drain(500);

    /* Two solid frames, painted in order, leave the panel solid in frame B's
       colour. Any pixel that is neither colour is a sheared row; any frame-A
       pixel left visible is a band of the previous picture. */
    size_t a = panel_count(0x11, 0x22);
    size_t b = panel_count(0x33, 0x44);
    CHECK(a + b == 240u * 240u, "every panel pixel belongs to one of the two frames");
    CHECK(a == 0, "and frame A is fully painted over, leaving no band behind");
    CHECK(fake_panel_oob == 0, "no write runs past the window it was opened for");
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
    case_a_delta_tile_reaches_the_panel();
    case_a_tile_split_across_chunks();
    case_many_tiles_in_one_frame();
    case_a_tile_that_would_overhang_is_refused();
    case_a_bad_tile_size_is_refused();
    case_a_truncated_tile_is_not_drawn();
    case_a_tile_missing_its_first_chunk_is_refused();
    case_tiles_before_a_keyframe_are_dropped();
    case_img_end_does_not_overtake_the_ring();
    case_a_repeated_chunk_is_refused();
    case_a_chunk_arriving_out_of_order_is_refused();
    case_a_scaled_keyframe_fills_the_panel();
    case_a_scale_that_would_overflow_is_refused();
    case_scale_zero_means_one_to_one();
    case_tiles_are_refused_while_scaled();
    case_a_new_keyframe_does_not_discard_the_one_still_queued();
    case_a_keyframe_starting_mid_paint_does_not_corrupt_the_panel();

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
