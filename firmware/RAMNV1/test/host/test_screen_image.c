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
#include "ramn_test_vectors.h"

/* ENABLE_SCREEN comes from the Makefile (CFLAGS_A), as it does in the real build. */
#include "../../Core/Src/ramn_screen_image.c"

#if !defined(RAMN_RLE_VECTOR_COUNT) || !defined(RAMN_PIPE_VECTOR_COUNT)
#error "vendored ramn_test_vectors.h predates the RLE or pipeline vectors -- refresh it"
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
    tileAssemblyPos   = 0;
    tileReady         = False;
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

       Marked as a known defect: the assertion states what a correct receiver
       would produce. When reassembly lands, this starts passing and the
       harness will say so. */
    reset_state();
    send_img_start(240, 240, 2, 100);

    /* Run of two pixels 0xABCD, with the control byte alone in frame 0 and
       its pixel bytes at the head of frame 1. */
    const uint8_t first[1]  = {0x81};
    const uint8_t second[2] = {0xAB, 0xCD};
    send_img_data(0, first, 1, 101);
    send_img_data(1, second, 2, 102);
    drain(103);

    CHECK_BUG(fake_screen_len == 4, "the split run still produces two pixels",
              "each frame is RLE_Decode'd on its own, so the control byte is"
              " dropped and the pixel bytes are read as a literal header."
              " Fix is to reassemble the stream before decoding (A-05).");
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

    CHECK_BUG(fake_screen_len == sizeof raw, "one full screen of pixels reaches the panel",
              "per-frame decoding loses every block that straddles a chunk"
              " boundary; measured at roughly two thirds of a screen, most of"
              " it misaligned. Fix is reassembly before decode (A-05).");
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

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
