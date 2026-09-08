/*
 * ramn_screen_image.c
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

// ECU A image streaming screen.
// Receives keyframe (0x300-0x303) and delta tile (0x304-0x306) CAN-FD frames,
// decodes RLE pixel data, and writes directly to the ST7789 display.
// No framebuffer: each tile/chunk is decoded and written to the screen as it arrives.

#include "ramn_screen_image.h"

#ifdef ENABLE_SCREEN

#include "ramn_config.h"
#include "ramn_spi.h"
#include "ramn_canfd.h"
#include "ramn_utils.h"
#include "ramn_uart.h"
#include <stdio.h>

// Define to enable UART debug output for image screen state transitions.
// Comment out to disable.
#define SCREENIMAGE_DEBUG

// RAMN_UART_SendFromTask is where this output goes, and ENABLE_UART is defined
// for TARGET_ECUD only (see ramn_config.h). This screen module runs on ECU A, which
// has no UART: the function is neither declared nor compiled there, so leaving
// the flag on makes every debug call an implicit declaration that fails at
// link. Turn the flag off where there is nothing to print to, rather than
// wrapping each call site.
#if defined(SCREENIMAGE_DEBUG) && !defined(ENABLE_UART)
#undef SCREENIMAGE_DEBUG
#endif

volatile RAMN_Bool_t RAMN_SCREENIMAGE_DisplayRequested = False;

// ============================================================================
// STATE MACHINE
// ============================================================================
typedef enum { IMG_IDLE, KEYFRAME_RX, IMG_SHOWN } ImageState_t;
static ImageState_t imgState = IMG_IDLE;

// Keyframe metadata
static uint16_t kfWidth        = 0;
static uint16_t kfHeight       = 0;

// Source-pixel repeat factor, from byte 8 of IMG_START. The sender ships a
// genuinely smaller image and the panel is filled here, which costs the link
// scale^2 times less than sending 240x240 with each block painted flat and
// hoping the RLE finds the runs -- it only partly does. Measured on a
// photographic frame: 3x3 flattened at full size is 824 chunks, the same
// picture sent as 80x80 and scaled here is 206.
//
// 1 means send-as-is and keeps the original streaming path untouched.
static uint8_t  kfScale        = 1U;   // the DRAIN's scale: latched at the ring's START entry
// The CAN RX task's own copy, latched where IMG_START arrives. Delta tiles are
// refused on that task, so its gate needs the scale of the frame ARRIVING, not
// the one the drain is still painting with.
static uint8_t  rxScale        = 1U;
// One source row is assembled before anything reaches the panel: the decoder
// emits an arbitrary run of bytes with no notion of where a row ends, and the
// ST7789's auto-increment only lands correctly if each expanded row is written
// whole and in order.
static uint8_t  kfRowBuf[LCD_WIDTH * 2];
static uint8_t  kfRowOut[LCD_WIDTH * 2];
static uint16_t kfRowFill      = 0;
static uint16_t kfTotalChunks  = 0;
static uint8_t  kfXOffset      = 0;
static uint8_t  kfYOffset      = 0;

// Bytes handed to the panel for the current keyframe. A 240x240 RGB565 frame
// is 115,200 bytes, so this MUST be wider than 16 bits: as a uint16_t it wrapped
// at 65,536 and could never report a complete keyframe.
static uint32_t kfDecodedBytes = 0;

// ---------------------------------------------------------------------------
// DIAGNOSTIC COUNTERS -- reported to ECU D in the 0x303 ACK
//
// ECU A has no UART (ENABLE_UART is TARGET_ECUD only), so the ACK is the only
// channel it has to say what actually happened to a keyframe. Every one of
// these counts a place where a chunk used to disappear in silence.
// ---------------------------------------------------------------------------
static uint16_t kfFramesRx   = 0;   // 0x301 frames accepted into the ring
static uint16_t kfRingDrops  = 0;   // 0x301 frames dropped: ring full
static uint16_t kfStateDrops = 0;   // 0x301 frames dropped: not in KEYFRAME_RX

// The previous keyframe's totals, captured just before IMG_START clears them.
// The START ack is the one ECU D reliably receives, so it carries these: an END
// ack that never arrives cannot report anything, and "nothing" is the case we
// most need described.
// (the previous frame's numbers now live in prevStats, below, filled in by the
// drain when it reaches an IMG_END rather than by whoever happens to run next)

// Screen hold state — mirrors the regcode pattern.
// screenActive is the authoritative "stay on screen" flag.
// It is set when a keyframe begins and cleared only on timeout or user navigation.
// This prevents Deinit (called on a RequestRedraw switchScreen) from accidentally
// dismissing the screen by clearing DisplayRequested mid-stream.
static RAMN_Bool_t screenActive     = False;
static uint32_t    screenActivatedTick = 0;
#define IMAGE_HOLD_MS    5000U   // How long to hold the screen after last activity

// Activity timeout — return to idle after IMAGE_HOLD_MS with no image traffic
static uint32_t lastActivityTick = 0;
#define IMAGE_TIMEOUT_MS IMAGE_HOLD_MS

// ============================================================================
// RING BUFFER — keyframe CAN chunks → SPI
// Single-producer (ReceiveCAN task) / single-consumer (Periodic Update task).
// No mutex needed; __DMB() barriers ensure correct ordering.
// ============================================================================
// Sized to absorb a whole keyframe's burst, because the producer is far faster
// than the consumer and the two are not rate-matched:
//
//   arrival  ECU D polls the ESP32 every 1 ms while streaming and each poll
//            yields two chunks, so ~2 frames/ms -- a 25-chunk keyframe lands in
//            roughly 13 ms.
//   drain    SCREENIMAGE_Update runs on the periodic task every 10 ms, and
//            writing a full frame to the panel is ~115,200 bytes at ~27 MHz,
//            about 33 ms.
//
// At 8 entries (7 usable) that dropped well over half of every keyframe. A
// dropped chunk is not a local hole: the RLE stream is one continuous stream,
// so losing one desynchronises every byte after it AND shifts the panel's
// auto-increment position, which is why a partly-received keyframe shows as
// nothing recognisable rather than a partial image.
//
// 64 entries covers the 25-45 chunk keyframes this canvas produces. A less
// compressible image can still overflow it -- that is what the ring-drop count
// in the 0x303 ACK is for.
#define KFRING_ENTRIES   64
#define KFRING_PAYLOAD  62    // bytes per IMG_DATA frame (62 bytes of RLE payload)

// Delta tile chunks go through THIS ring too, rather than a staging slot of
// their own. One queue, one consumer, and the decode happens where the SPI
// writes happen -- which is what makes streaming decode possible at all.
#define KFRING_KIND_IMG   0U   // keyframe chunk  (0x301)
#define KFRING_KIND_TILE  1U   // delta tile chunk (0x305)
// IMG_END rides the ring rather than being answered where it arrives. It is
// handled on the CAN RX task; the chunks it terminates are decoded on the 10 ms
// periodic task, and nothing else orders the two. Answered on arrival it
// overtakes its own frame: the ACK measures whatever happened to be drained,
// and imgState reaches IMG_SHOWN while the panel is still being written.
#define KFRING_KIND_END   2U   // end of keyframe  (0x302) + the RX task's counters
// IMG_START rides the ring for the same reason IMG_END does, and it is the more
// dangerous of the two. It used to be applied where it arrived, on the CAN RX
// task, which meant it zeroed kfRingWriteIdx and kfRingReadIdx out from under a
// drain that was still painting the previous keyframe. Two things follow, both
// measured in test_screen_image.c:
//
//   between paints  every entry the previous frame left in the ring, its
//                   IMG_END included, is discarded by the producer. The frame
//                   is not torn or partial -- it never reaches the glass. One
//                   whole picture in two goes missing.
//
//   during a paint  the drain holds a local copy of the read index across a
//                   blocking SPI write (~29 ms for a full panel), so zeroing
//                   the indices leaves it walking entries the producer has
//                   begun refilling. It decodes the OLD frame's bytes with the
//                   NEW frame's geometry, into the new frame's window: 168 rows
//                   of the previous picture, 72 rows never written, and the new
//                   frame nowhere. Nothing counts it; the only symptom is on
//                   the glass.
//
// The paint is ~29 ms regardless of how small the source is, because the panel
// is always 240x240 -- so the lower the resolution, the larger the share of
// each frame period spent inside that window. That is why this shows up at
// 60x60 and not at 240x240.
//
// Queued instead, the frame boundary is just another entry in the same FIFO:
// the previous keyframe finishes painting into its own window before the next
// one opens, because that is what the order in the ring means.
//
//   data[0..1] width   data[2..3] height  data[4..5] total chunks
//   data[6] x offset   data[7] y offset   data[8] scale
//
// It also carries the OUTGOING frame's arrival counters, snapshotted just
// before they are zeroed. A keyframe whose IMG_END never arrived still has to
// be reportable -- the START ack exists precisely because the END ack is the
// one that goes missing -- and by the time the drain reaches this entry the CAN
// RX task has long since reset those counters for the new frame.
//
//   data[9..10] framesRx    data[11..12] ringDrops  data[13..14] stateDrops
//   data[15..16] dupSkips   data[17] seqBroken
#define KFRING_KIND_START 3U

typedef struct {
    uint8_t data[KFRING_PAYLOAD];
    uint8_t len;
    uint8_t kind;
    uint8_t tileX;      // tile fields are meaningful only for KFRING_KIND_TILE
    uint8_t tileY;
    uint8_t tileSize;
    uint8_t tileSeq;
} KFRingEntry_t;

static KFRingEntry_t        kfRingBuf[KFRING_ENTRIES];
static volatile uint8_t     kfRingWriteIdx = 0;
static volatile uint8_t     kfRingReadIdx  = 0;

// The keyframe RLE stream is one continuous byte sequence cut at fixed offsets,
// so a chunk decoded twice shifts every block after it exactly as a lost one
// does. SEQ_HI/SEQ_LO were read off each 0x301 frame and then ignored, which
// left the decoder trusting arrival order on a link that does not guarantee it:
// the ESP32 re-presents its staged SPI response whenever ECU D's write did not
// parse as a poll, and on a full-duplex bus ECU D has already clocked those
// bytes out and forwarded them. Checking the sequence here makes a repeat free
// and a gap loud, so the link is allowed to be redundant but never silent.
// Defined below with the ACK layout it builds; the ring drain answers IMG_END
// and so needs it ahead of that.
#define IMG_ACK_START  0x02U
#define IMG_ACK_END    0x00U
#define IMG_ACK_LATE   0x03U

// What one keyframe is worth reporting. Passed in rather than read off globals
// because the two halves of it are counted on DIFFERENT TASKS: framesRx and the
// drop counters on the CAN RX task as frames arrive, decoded on the periodic
// task as they are painted. Once the frame boundary rides the ring, those two
// tasks are no longer at the same frame -- the RX task can already be counting
// frame N+1 while the drain is still answering IMG_END for frame N. Reading
// globals at that moment reports the wrong frame's numbers, which is how a
// diagnostic quietly starts lying at exactly the load where it is needed.
//
// So the RX task's counters travel WITH the IMG_END entry, snapshotted at the
// instant the frame ended, and the drain adds only what it owns.
typedef struct {
    uint32_t    decoded;      // bytes written to the panel (periodic task)
    uint16_t    framesRx;     // 0x301 accepted           (CAN RX task)
    uint16_t    ringDrops;    // 0x301 dropped, ring full (CAN RX task)
    uint16_t    stateDrops;   // 0x301 dropped, bad state (CAN RX task)
    uint16_t    dupSkips;     // 0x301 delivered twice    (CAN RX task)
    RAMN_Bool_t seqBroken;    // a chunk never arrived    (CAN RX task)
    RAMN_Bool_t truncated;    // a half-read RLE block at IMG_END
} ImgFrameStats_t;

static void SendImageAck(uint8_t stage, uint8_t endStatus, const ImgFrameStats_t* st);

// The last frame the drain finished, for the next START ack to report.
// Periodic task only.
static ImgFrameStats_t prevStats;
// Whether an IMG_END entry described that frame. A frame whose IMG_END was lost
// still has to be reported, and prevStats is the only place left to do it.
static RAMN_Bool_t     kfEndSeen = False;

static uint16_t             kfExpectedSeq = 0;   // CAN RX task only
// Written on the CAN RX task, read by the periodic task when it answers
// IMG_END, so the same volatile treatment as the ring indices.
static volatile RAMN_Bool_t kfSeqBroken   = False;   // a chunk is missing; frame is dead
static volatile uint16_t    kfDupSkips    = 0;       // chunks the link delivered twice

// ============================================================================
// TILE ASSEMBLY BUFFER — accumulates RLE-decoded bytes for one delta tile
// ============================================================================
#define TILE_RAW_MAX  (40 * 40 * 2)   // 3,200 bytes — worst-case 40×40 RGB565

static uint8_t  tileAssemblyBuf[TILE_RAW_MAX];
static uint16_t tileAssemblyPos = 0;

// Current tile being assembled
static uint8_t curTileX    = 0;
static uint8_t curTileY    = 0;
static uint8_t curTileSize = 0;   // pixel width/height: 8, 16, or 40
// True between a tile's first chunk and its last. Continuation chunks are only
// believed while a tile is open AND their coordinates match the open one.
static RAMN_Bool_t tileActive = False;

// Counts tile chunks refused before they reach the ring: an impossible size, a
// tile that would hang off the edge of the panel, or a length that disagrees
// with the frame it arrived in.
static uint16_t kfTileDrops = 0;
// Tiles whose chunks stopped arriving before the tile was complete.
static uint16_t kfTileShort = 0;

// ============================================================================
// DEFERRED SPI WORK — set by ProcessRxCANMessage (CAN RX task), consumed by
// Update (Periodic task). SPI functions block on ulTaskNotifyTake and must
// only be called from the Periodic task registered with RAMN_SPI_Init.
// ============================================================================

// The keyframe window is opened by the KFRING_KIND_START entry as the drain
// reaches it, so there is no deferred-window flag any more. A flag was a second
// channel carrying the frame boundary alongside the ring, and the two could not
// be kept in order: the flag was applied at the top of Update, ahead of chunks
// still queued from the previous frame.

// Delta tiles are assembled and written by Update straight out of the ring, so
// there is no staging copy and no single slot to overflow. The old design
// staged one finished tile at a time and dropped any tile that arrived while
// the previous one was still waiting for the 10 ms periodic task -- which is
// most of a delta frame.

// Static decode buffer for keyframe ring-buffer drain — avoids large stack frame.
// Worst case: one run carried in from the previous frame (128 px = 256 bytes),
// plus 61 RLE bytes of this one = 20 repeat-runs × 128 repetitions × 2 bytes
// = 5120. 5376 total; round up.
#define KF_DECODE_BUF_SIZE  5440U
static uint8_t kfDecodeBuf[KF_DECODE_BUF_SIZE];

// One byte of a pixel whose partner has not been decoded yet. The panel only
// accepts whole pixels, so an odd tail waits here for the next chunk.
static uint8_t         kfCarryByte  = 0;
static volatile RAMN_Bool_t kfCarryValid = False;

// ============================================================================
// RLE DECODER
// ============================================================================
// Control byte layout:
//   Bit 7 = 0: LITERAL run  — next (N+1) bytes are literal (N = bits 6..0)
//   Bit 7 = 1: REPEAT run   — next 2 bytes (one RGB565 pixel LE) repeated N+1 times
// Operates on raw bytes, not pixels. Tiles are always even-byte-sized.
// Returns number of bytes written to dst.
// ============================================================================
// ---------------------------------------------------------------------------
// Streaming decoder: one RLE stream spread across many CAN frames
// ---------------------------------------------------------------------------
//
// A keyframe is RLE-encoded as ONE stream over the whole image and only then
// cut into fixed-size chunks, so a block routinely begins in one frame and
// ends in the next: measured on a real 240x240 keyframe, 211 of 286 chunk
// boundaries have a block straddling them.
//
// Decoding each frame on its own therefore cannot work, and fails quietly --
// a run whose control byte ends one frame produces nothing from that frame,
// and its two pixel bytes are read as a literal header at the start of the
// next. Roughly two thirds of a screen came out, most of it misaligned.
//
// This carries the partial block across the boundary instead. State is one
// control byte, at most two pixel bytes, and two counters -- not a copy of the
// stream, which matters because ECU A has no framebuffer and decodes straight
// to the panel.
//
// Emission is resumable in both directions: if dst fills mid-run the remaining
// repetitions stay in runLeft and come out on the next call, before any new
// source byte is read.

typedef struct {
    uint16_t runLeft;   // pixel repetitions still to emit
    uint16_t litLeft;   // literal bytes still to copy
    uint16_t runCount;  // repetitions this run will emit once its pixel arrives
    uint8_t  pix[2];    // the run's pixel, as much of it as has arrived
    uint8_t  pixHave;   // 0..2
    uint8_t  awaitPix;  // a run control byte was read; its pixel has not
} RleStream_t;

static RleStream_t kfStream;

// A delta tile is one RLE stream cut into 59-byte chunks by ECU D, exactly as a
// keyframe is cut into 61-byte ones. A 40x40 tile is 3,200 raw bytes and needs
// up to ~55 chunks, so blocks straddle chunk boundaries constantly. Decoding
// each chunk on its own -- which is what this module used to do -- loses every
// block that spans a boundary, silently. Same defect as the keyframe path had,
// same fix: carry the partial block across.
static RleStream_t tileStream;

static void RLE_StreamReset(RleStream_t* s)
{
    s->runLeft = 0U;
    s->litLeft = 0U;
    s->runCount = 0U;
    s->pix[0] = 0U;
    s->pix[1] = 0U;
    s->pixHave = 0U;
    s->awaitPix = 0U;
}

// True when a block is half-read -- at IMG_END this means the stream was
// truncated, which is worth counting rather than ignoring.
static RAMN_Bool_t RLE_StreamMidBlock(const RleStream_t* s)
{
    return (s->runLeft || s->litLeft || s->awaitPix) ? True : False;
}

static uint16_t RLE_DecodeStream(RleStream_t* s, const uint8_t* src, uint16_t srcLen,
                                 uint8_t* dst, uint16_t dstMax)
{
    uint16_t si = 0U, di = 0U;

    for (;;)
    {
        // 1. Finish a run left over from a previous frame or a full dst.
        if (s->runLeft)
        {
            while (s->runLeft && (di + 2U) <= dstMax)
            {
                dst[di++] = s->pix[0];
                dst[di++] = s->pix[1];
                s->runLeft--;
            }
            if (s->runLeft) break;          // dst full; resume next call
        }

        // 2. Finish a literal, which may span the frame boundary.
        if (s->litLeft)
        {
            while (s->litLeft && si < srcLen && di < dstMax)
            {
                dst[di++] = src[si++];
                s->litLeft--;
            }
            if (s->litLeft) break;          // needs more source, or more dst
        }

        // 3. Collect the pixel of a run whose control byte already arrived.
        if (s->awaitPix)
        {
            while (s->pixHave < 2U && si < srcLen) s->pix[s->pixHave++] = src[si++];
            if (s->pixHave < 2U) break;     // the rest is in the next frame
            s->runLeft  = s->runCount;
            s->awaitPix = 0U;
            s->pixHave  = 0U;
            continue;
        }

        // 4. Start a new block.
        if (si >= srcLen || di >= dstMax) break;
        uint8_t  ctrl = src[si++];
        uint16_t n    = (uint16_t)((ctrl & 0x7FU) + 1U);
        if (ctrl & 0x80U)
        {
            s->runCount = n;
            s->awaitPix = 1U;
            s->pixHave  = 0U;
        }
        else
        {
            s->litLeft = n;
        }
    }

    return di;
}

static uint16_t RLE_Decode(const uint8_t* src, uint16_t srcLen,
                            uint8_t* dst, uint16_t dstMax)
{
    uint16_t si = 0, di = 0;

    while (si < srcLen && di < dstMax)
    {
        uint8_t ctrl = src[si++];
        uint8_t n    = (uint8_t)((ctrl & 0x7FU) + 1U);

        if (ctrl & 0x80U)
        {
            // Repeat run: 2 bytes (one pixel) repeated n times
            if (si + 1U >= srcLen) break;
            uint8_t b0 = src[si++];
            uint8_t b1 = src[si++];
            for (uint8_t i = 0; i < n && (di + 1U) < dstMax; i++)
            {
                dst[di++] = b0;
                dst[di++] = b1;
            }
        }
        else
        {
            // Literal run: copy n bytes verbatim
            for (uint8_t i = 0; i < n && si < srcLen && di < dstMax; i++)
            {
                dst[di++] = src[si++];
            }
        }
    }

    return di;
}

// ============================================================================
// SCREEN CALLBACKS
// ============================================================================

static void SCREENIMAGE_Init(void)
{
#ifdef SCREENIMAGE_DEBUG
    char buf[64];
    int  len = snprintf(buf, sizeof(buf), "IMG Init: active=%d req=%d state=%d\r\n",
        (int)screenActive, (int)RAMN_SCREENIMAGE_DisplayRequested, (int)imgState);
    if (len > 0) RAMN_UART_SendFromTask((uint8_t*)buf, (uint32_t)len);
#endif
    kfScale   = 1U;
    rxScale   = 1U;
    kfRowFill = 0U;
    RAMN_SPI_DrawRectangle(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);
    RAMN_SPI_DrawString(50, 108, COLOR_WHITE, COLOR_BLACK, "  Streaming...");
}

static void SCREENIMAGE_Deinit(void)
{
#ifdef SCREENIMAGE_DEBUG
    char buf[64];
    int  len = snprintf(buf, sizeof(buf), "IMG Deinit: active=%d req=%d state=%d\r\n",
        (int)screenActive, (int)RAMN_SCREENIMAGE_DisplayRequested, (int)imgState);
    if (len > 0) RAMN_UART_SendFromTask((uint8_t*)buf, (uint32_t)len);
#endif
    // Only fully clear DisplayRequested if we are not actively holding the screen.
    // If screenActive is True, Deinit was triggered by a RequestRedraw switchScreen —
    // keep DisplayRequested set so the screen manager switches back immediately.
    if (screenActive == False)
    {
        RAMN_SCREENIMAGE_DisplayRequested = False;
    }

    imgState        = IMG_IDLE;
    RLE_StreamReset(&kfStream);
    kfCarryValid    = False;
    kfRingWriteIdx  = 0;
    kfRingReadIdx   = 0;
    kfDecodedBytes  = 0;
    kfFramesRx      = 0;
    kfRingDrops     = 0;
    kfStateDrops    = 0;
    tileAssemblyPos = 0;
    kfEndSeen       = False;
    kfTileDrops     = 0;
    kfTileShort     = 0;
    tileActive      = False;
    RLE_StreamReset(&tileStream);
}

// Repeat every source pixel kfScale times across and kfScale times down.
//
// Called only with kfScale > 1 and an even byte count, from the periodic task
// -- RAMN_SPI_WriteImageChunk blocks on the DMA's task notification, so
// writing the same row buffer several times in a row is safe.
static void WriteScaledPixels(const uint8_t* px, uint16_t len)
{
    const uint16_t rowBytes = (uint16_t)(kfWidth * 2U);
    uint16_t i = 0U;

    while (i < len)
    {
        uint16_t take = (uint16_t)(rowBytes - kfRowFill);
        if (take > (uint16_t)(len - i)) take = (uint16_t)(len - i);
        for (uint16_t k = 0U; k < take; k++) kfRowBuf[kfRowFill + k] = px[i + k];
        kfRowFill = (uint16_t)(kfRowFill + take);
        i         = (uint16_t)(i + take);

        if (kfRowFill < rowBytes) break;      // still short of a whole row

        uint16_t o = 0U;
        for (uint16_t sx = 0U; sx < rowBytes; sx += 2U)
        {
            for (uint8_t r = 0U; r < kfScale; r++)
            {
                kfRowOut[o]      = kfRowBuf[sx];
                kfRowOut[o + 1U] = kfRowBuf[sx + 1U];
                o = (uint16_t)(o + 2U);
            }
        }
        for (uint8_t r = 0U; r < kfScale; r++) RAMN_SPI_WriteImageChunk(kfRowOut, o);
        kfRowFill = 0U;
    }
}

static void SCREENIMAGE_Update(uint32_t tick)
{
    // ---- Drain keyframe ring buffer → RLE decode → SPI ----
    // Runs in both KEYFRAME_RX and IMG_SHOWN (to flush trailing chunks after IMG_END).
    {
        uint8_t ri = kfRingReadIdx;
        __DMB();
        while (ri != kfRingWriteIdx)
        {
            KFRingEntry_t* entry = &kfRingBuf[ri];

            if (entry->kind == KFRING_KIND_START)
            {
                // Reaching this entry is what "the previous keyframe is
                // finished" means: every chunk of it is ahead in the ring and
                // has already been decoded and written. Only now is it safe to
                // repoint the decoder at a new picture.
                //
                // If no IMG_END entry came through for that frame, nothing has
                // described it yet -- and an unreported frame is the case the
                // START ack was added for. Build its numbers here instead,
                // from what this task painted plus the arrival counters the
                // entry carries.
                if (kfEndSeen == False)
                {
                    prevStats.decoded    = kfDecodedBytes;
                    prevStats.framesRx   = (uint16_t)((uint16_t)entry->data[9]  |
                                                     ((uint16_t)entry->data[10] << 8));
                    prevStats.ringDrops  = (uint16_t)((uint16_t)entry->data[11] |
                                                     ((uint16_t)entry->data[12] << 8));
                    prevStats.stateDrops = (uint16_t)((uint16_t)entry->data[13] |
                                                     ((uint16_t)entry->data[14] << 8));
                    prevStats.dupSkips   = (uint16_t)((uint16_t)entry->data[15] |
                                                     ((uint16_t)entry->data[16] << 8));
                    prevStats.seqBroken  = (entry->data[17] != 0U) ? True : False;
                    prevStats.truncated  = RLE_StreamMidBlock(&kfStream);
                }
                kfEndSeen = False;

                SendImageAck(IMG_ACK_START, 0U, &prevStats);

                kfWidth       = (uint16_t)((uint16_t)entry->data[0] |
                                          ((uint16_t)entry->data[1] << 8));
                kfHeight      = (uint16_t)((uint16_t)entry->data[2] |
                                          ((uint16_t)entry->data[3] << 8));
                kfTotalChunks = (uint16_t)((uint16_t)entry->data[4] |
                                          ((uint16_t)entry->data[5] << 8));
                kfXOffset     = entry->data[6];
                kfYOffset     = entry->data[7];
                kfScale       = entry->data[8];

                kfDecodedBytes = 0;
                kfRowFill      = 0U;
                kfCarryValid   = False;
                RLE_StreamReset(&kfStream);   // a new keyframe starts a new stream

                tileAssemblyPos = 0;
                tileActive      = False;
                RLE_StreamReset(&tileStream);

                RAMN_SPI_OpenImageWindow(kfXOffset, kfYOffset,
                                         (uint16_t)(kfWidth  * kfScale),
                                         (uint16_t)(kfHeight * kfScale));

                ri = (ri + 1U) % KFRING_ENTRIES;
                __DMB();
                kfRingReadIdx = ri;
                continue;
            }

            if (entry->kind == KFRING_KIND_END)
            {
                // Every chunk ahead of this entry has now been decoded and
                // written, so these numbers describe the whole frame. The
                // arrival half of them was snapshotted into the entry when the
                // frame ended; only `decoded` is this task's to add.
                ImgFrameStats_t st;
                st.decoded    = kfDecodedBytes;
                st.framesRx   = (uint16_t)((uint16_t)entry->data[1] |
                                          ((uint16_t)entry->data[2] << 8));
                st.ringDrops  = (uint16_t)((uint16_t)entry->data[3] |
                                          ((uint16_t)entry->data[4] << 8));
                st.stateDrops = (uint16_t)((uint16_t)entry->data[5] |
                                          ((uint16_t)entry->data[6] << 8));
                st.dupSkips   = (uint16_t)((uint16_t)entry->data[7] |
                                          ((uint16_t)entry->data[8] << 8));
                st.seqBroken  = (entry->data[9] != 0U) ? True : False;
                st.truncated  = ((RLE_StreamMidBlock(&kfStream) != False) ||
                                 (st.seqBroken != False)) ? True : False;

                SendImageAck(IMG_ACK_END, entry->data[0], &st);
                prevStats = st;               // for the next frame's START ack
                kfEndSeen = True;
                imgState  = IMG_SHOWN;

                ri = (ri + 1U) % KFRING_ENTRIES;
                __DMB();
                kfRingReadIdx = ri;
                continue;
            }

            if (entry->kind == KFRING_KIND_TILE)
            {
                // First chunk of a tile: latch its geometry and start a fresh
                // RLE stream. Bit 7 of the sequence marks the last chunk.
                if ((entry->tileSeq & 0x7FU) == 0U)
                {
                    curTileX    = entry->tileX;
                    curTileY    = entry->tileY;
                    curTileSize = entry->tileSize;
                    tileAssemblyPos = 0U;
                    tileActive  = True;
                    RLE_StreamReset(&tileStream);
                }
                else if ((tileActive == False) ||
                         (entry->tileX    != curTileX) ||
                         (entry->tileY    != curTileY) ||
                         (entry->tileSize != curTileSize))
                {
                    // A continuation chunk whose geometry does not match the
                    // tile in progress: this tile's FIRST chunk was lost. Every
                    // chunk carries its own coordinates, so decoding it into
                    // whatever tile happened to be open paints it at the wrong
                    // place -- confidently, with no way to tell afterwards.
                    // Nothing drops on the host, so only a lossy bus gets here.
                    if (kfTileDrops < 0xFFFFU) kfTileDrops++;
                    tileActive = False;
                    ri = (ri + 1U) % KFRING_ENTRIES;
                    __DMB();
                    kfRingReadIdx = ri;
                    continue;
                }

                uint16_t room = (uint16_t)(TILE_RAW_MAX - tileAssemblyPos);
                uint16_t got  = RLE_DecodeStream(&tileStream, entry->data, entry->len,
                                                 &tileAssemblyBuf[tileAssemblyPos], room);
                tileAssemblyPos = (uint16_t)(tileAssemblyPos + got);

                if (entry->tileSeq & 0x80U)
                {
                    uint16_t w    = (uint16_t)curTileSize;
                    uint16_t need = (uint16_t)(w * w * 2U);
                    if (tileAssemblyPos >= need)
                    {
                        RAMN_SPI_OpenImageWindow((uint16_t)((uint16_t)curTileX * 8U),
                                                 (uint16_t)((uint16_t)curTileY * 8U), w, w);
                        RAMN_SPI_WriteImageChunk(tileAssemblyBuf, need);
                        kfDecodedBytes += need;
                    }
                    else if (kfTileShort < 0xFFFFU)
                    {
                        // Chunks stopped before the tile was whole. Writing a
                        // partial tile would shift every row inside it, so the
                        // tile is left as it was and the miss is counted.
                        kfTileShort++;
                    }
                    tileAssemblyPos = 0U;
                    tileActive = False;
                }

                ri = (ri + 1U) % KFRING_ENTRIES;
                __DMB();
                kfRingReadIdx = ri;
                continue;
            }

            // Decode into static buffer — avoids large stack frame and ensures
            // sufficient capacity. RLE_DecodeStream carries a block that
            // straddles the frame boundary; decoding each frame on its own
            // silently loses every such block, and on a real keyframe most
            // boundaries have one.
            // A byte held back from the previous chunk goes out in front of
            // this one, so the panel always receives whole pixels.
            uint16_t off = 0U;
            if (kfCarryValid != False) { kfDecodeBuf[0] = kfCarryByte; off = 1U; }

            uint16_t decodedLen = RLE_DecodeStream(&kfStream, entry->data, entry->len,
                                                   &kfDecodeBuf[off],
                                                   (uint16_t)(KF_DECODE_BUF_SIZE - off));
            decodedLen = (uint16_t)(decodedLen + off);
            kfCarryValid = False;

            // RAMN_SPI_WriteImageChunk DISCARDS an odd-length write outright --
            // it is a byte stream to us but pixels to the ST7789. A literal can
            // end mid-pixel, so odd lengths are normal here and handing one over
            // loses the whole chunk in silence. Hold the trailing byte back and
            // send it with the next one instead.
            if (decodedLen & 1U)
            {
                kfCarryByte  = kfDecodeBuf[decodedLen - 1U];
                kfCarryValid = True;
                decodedLen--;
            }

            if (decodedLen > 0U)
            {
                if (kfScale > 1U) WriteScaledPixels(kfDecodeBuf, decodedLen);
                else              RAMN_SPI_WriteImageChunk(kfDecodeBuf, decodedLen);
                kfDecodedBytes += decodedLen;
            }

            ri = (ri + 1U) % KFRING_ENTRIES;
            __DMB();
            kfRingReadIdx = ri;
        }
    }

    // ---- Timeout: hold screen for IMAGE_HOLD_MS after last activity, then dismiss ----
    //
    // TWO DIFFERENT CLOCKS used to meet here, and the mismatch dismissed the
    // screen on the first update of every keyframe.
    //
    // lastActivityTick comes from the CAN RX task as xTaskGetTickCount() -- real
    // time now. The `tick` argument is the periodic task's xLastWakeTime, which
    // vTaskDelayUntil advances by exactly SIM_LOOP_CLOCK_MS per iteration. Writing
    // a keyframe to the panel is ~115,200 bytes, about 33 ms inside a 10 ms
    // period, so that loop overruns hard and xLastWakeTime falls PERMANENTLY
    // behind real time -- it never catches up.
    //
    // Once it lags at all, `tick` is less than lastActivityTick, the unsigned
    // subtraction wraps to ~4.29 billion, and the comparison against 5,000 is
    // always true. The screen was torn down mid-keyframe on every frame: Deinit
    // set imgState back to IMG_IDLE, every remaining chunk was rejected as
    // out-of-state, and IMG_END came back as an IMG_ACK_LATE with decoded=0.
    //
    // Read the same clock the activity timestamp was taken from, and compare as
    // a SIGNED difference so a lagging or wrapped tick can never dismiss.
    uint32_t nowTick   = (uint32_t)xTaskGetTickCount();
    int32_t  sinceLast = (int32_t)(nowTick - lastActivityTick);
    if (screenActive && (sinceLast > (int32_t)IMAGE_HOLD_MS))
    {
        screenActive                   = False;
        imgState                       = IMG_IDLE;
        RAMN_SCREENIMAGE_DisplayRequested = False;
    }
}

static RAMN_Bool_t SCREENIMAGE_UpdateInput(JoystickEventType event)
{
    if (screenActive)
    {
        // Any left/right joystick press exits — mirrors regcode behaviour
        if (event == JOYSTICK_EVENT_LEFT_PRESSED || event == JOYSTICK_EVENT_RIGHT_PRESSED)
        {
            screenActive                   = False;
            imgState                       = IMG_IDLE;
            RAMN_SCREENIMAGE_DisplayRequested = False;
            return True;   // let screen manager handle navigation
        }
        return False;  // block navigation while screen is held
    }
    return True;   // allow navigation when screen is not active
}

// ============================================================================
// 0x303 ACK -- ECU A's only report channel
//
// ECU A has no UART (ENABLE_UART is TARGET_ECUD only), so this frame is the
// only way it can say anything about a keyframe. It is sent twice per frame:
// once on IMG_START and once on IMG_END, with a different byte 0, so a silent
// receiver can be told apart from one that gets IMG_START and never sees
// IMG_END.
//
//   [0] stage/status : IMG_ACK_START (0x02) = 0x300 seen, keyframe begun.
//                            Bytes 2..6 then describe the PREVIOUS keyframe.
//                      0x00 = keyframe complete and clean
//                      0x01 = keyframe finished with a problem (see flags)
//                      IMG_ACK_LATE (0x03) = 0x302 arrived but this ECU was not
//                            in KEYFRAME_RX. Sent anyway: an unanswered IMG_END
//                            and an IMG_END that never arrived are the same
//                            silence otherwise, and they need different fixes.
//   [1] flags        : bit0 truncated (a half-read RLE block at IMG_END)
//                      bit1 chunks dropped, ring full
//                      bit2 chunks dropped, wrong state
//                      bit3 a chunk never arrived; the frame was abandoned
//                      bit4 the link delivered a chunk twice (harmless: the
//                           repeat was skipped, not decoded)
//   [2..4]           : bytes written to the panel, 24-bit little-endian
//                      (a full 240x240 keyframe is 115,200)
//   [5]              : 0x301 frames accepted                (saturating 255)
//   [6]              : 0x301 frames dropped, ring full       (saturating)
//   [7]              : FDCAN RX overruns since boot          (saturating)
//                      -- frames the peripheral dropped before this module saw
//                      them. 23 back-to-back 64-byte FD frames arrive in about
//                      1.5 ms, so this is where a burst is lost if it is lost.
//
// Its own function so that a second call site does not grow
// SCREENIMAGE_ProcessRxCANMessage's frame: that runs on the CAN RX task, whose
// whole stack is 1 KB (RAMN_ReceiveCANBuffer[256] in main.c).
// ============================================================================
static void SendImageAck(uint8_t stage, uint8_t endStatus, const ImgFrameStats_t* st)
{
    FDCAN_TxHeaderTypeDef ackHdr;
    ackHdr.Identifier          = IMG_CAN_ID_ACK;
    ackHdr.IdType              = FDCAN_STANDARD_ID;
    ackHdr.TxFrameType         = FDCAN_DATA_FRAME;
    ackHdr.DataLength          = FDCAN_DLC_BYTES_8;
    ackHdr.BitRateSwitch       = FDCAN_BRS_OFF;
    ackHdr.FDFormat            = FDCAN_CLASSIC_CAN;
    ackHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    ackHdr.MessageMarker       = 0U;
    ackHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

    uint8_t flags = 0U;
    if (st->truncated  != False) flags |= 0x01U;
    if (st->ringDrops  != 0U)    flags |= 0x02U;
    if (st->stateDrops != 0U)    flags |= 0x04U;
    if (st->seqBroken  != False) flags |= 0x08U;
    if (st->dupSkips   != 0U)    flags |= 0x10U;

    uint32_t decoded = st->decoded;
    uint16_t rx      = st->framesRx;
    uint16_t drops   = st->ringDrops;

    uint32_t overrun = RAMN_FDCAN_Status.CANRxOverrunCnt;

    uint8_t ackData[8];
    if (stage == IMG_ACK_START)     ackData[0] = IMG_ACK_START;
    else if (stage == IMG_ACK_LATE) ackData[0] = IMG_ACK_LATE;
    else ackData[0] = ((endStatus == 0x00U) && (flags == 0U)) ? 0x00U : 0x01U;
    ackData[1] = flags;
    ackData[2] = (uint8_t)(decoded & 0xFFU);
    ackData[3] = (uint8_t)((decoded >> 8) & 0xFFU);
    ackData[4] = (uint8_t)((decoded >> 16) & 0xFFU);
    ackData[5] = (rx      > 255U) ? 255U : (uint8_t)rx;
    ackData[6] = (drops   > 255U) ? 255U : (uint8_t)drops;
    ackData[7] = (overrun > 255U) ? 255U : (uint8_t)overrun;
    RAMN_FDCAN_SendMessage(&ackHdr, ackData);
}

static void SCREENIMAGE_ProcessRxCANMessage(const FDCAN_RxHeaderTypeDef* pHeader,
                                            const uint8_t* data, uint32_t tick)
{
    uint32_t id      = pHeader->Identifier;
    uint8_t  dlcLen  = DLCtoUINT8(pHeader->DataLength);

    lastActivityTick = tick;

    // ---- 0x300: IMG_START ----
    if (id == IMG_CAN_ID_START)
    {
        if (dlcLen < 12U) return;

        // Activate screen hold — mirrors regcode pattern
        screenActive        = True;
        screenActivatedTick = tick;
        lastActivityTick    = tick;

        // Geometry is PARSED here and LATCHED IN THE DRAIN. Nothing this handler
        // writes may be read by the drain for the frame it is still painting --
        // that is the whole point of putting the boundary in the ring.
        uint16_t w  = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        uint16_t h  = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
        uint16_t tc = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));

        // Clamp dimensions to physical screen size
        if (w > (uint16_t)LCD_WIDTH)  w = (uint16_t)LCD_WIDTH;
        if (h > (uint16_t)LCD_HEIGHT) h = (uint16_t)LCD_HEIGHT;

        // Byte 8 is the scale factor. A sender that predates it sends 0, and
        // a short frame has no byte 8 at all; both mean 1:1. Anything that
        // would not fit the panel is refused rather than clamped -- a wrong
        // scale does not crop the image, it shears every row after the first,
        // and a 1:1 picture in the corner is far easier to recognise as wrong.
        uint8_t sc = (dlcLen >= 9U) ? data[8] : 1U;
        if (sc == 0U) sc = 1U;
        if (((uint32_t)w * sc) > (uint32_t)LCD_WIDTH ||
            ((uint32_t)h * sc) > (uint32_t)LCD_HEIGHT)
        {
            sc = 1U;
        }

        // State this task owns outright: the sequence gate, the tile gate, and
        // the arrival counters. All of it is safe to reset here precisely
        // because the drain never reads it -- the drain gets its copy of these
        // numbers inside the IMG_END entry.
        uint16_t    outFramesRx   = kfFramesRx;
        uint16_t    outRingDrops  = kfRingDrops;
        uint16_t    outStateDrops = kfStateDrops;
        uint16_t    outDupSkips   = kfDupSkips;
        RAMN_Bool_t outSeqBroken  = kfSeqBroken;

        rxScale       = sc;
        kfExpectedSeq = 0;           // IMG_START is the only resynchronisation
        kfSeqBroken   = False;
        kfDupSkips    = 0;
        kfFramesRx    = 0;
        kfRingDrops   = 0;
        kfStateDrops  = 0;

        {
            uint8_t wi      = kfRingWriteIdx;
            uint8_t next_wi = (uint8_t)((wi + 1U) % KFRING_ENTRIES);
            if (next_wi == kfRingReadIdx)
            {
                // No room to order this behind the previous frame's chunks.
                // Letting the chunks through anyway decodes them into the
                // window the PREVIOUS keyframe opened, at the previous
                // geometry -- precisely the corruption this entry exists to
                // prevent. Kill the frame instead: the gate refuses every
                // chunk, IMG_END reports the gap flag, and the next IMG_START
                // recovers. A missing picture is recoverable and counted; a
                // sheared one that nothing counts is neither.
                kfSeqBroken = True;
                if (kfRingDrops < 0xFFFFU) kfRingDrops++;
            }
            else
            {
                KFRingEntry_t* e = &kfRingBuf[wi];
                e->data[0] = (uint8_t)(w  & 0xFFU);  e->data[1] = (uint8_t)(w  >> 8);
                e->data[2] = (uint8_t)(h  & 0xFFU);  e->data[3] = (uint8_t)(h  >> 8);
                e->data[4] = (uint8_t)(tc & 0xFFU);  e->data[5] = (uint8_t)(tc >> 8);
                e->data[6] = data[6];
                e->data[7] = data[7];
                e->data[8] = sc;
                e->data[9]  = (uint8_t)(outFramesRx   & 0xFFU);
                e->data[10] = (uint8_t)(outFramesRx   >> 8);
                e->data[11] = (uint8_t)(outRingDrops  & 0xFFU);
                e->data[12] = (uint8_t)(outRingDrops  >> 8);
                e->data[13] = (uint8_t)(outStateDrops & 0xFFU);
                e->data[14] = (uint8_t)(outStateDrops >> 8);
                e->data[15] = (uint8_t)(outDupSkips   & 0xFFU);
                e->data[16] = (uint8_t)(outDupSkips   >> 8);
                e->data[17] = (outSeqBroken != False) ? 1U : 0U;
                e->len     = 18U;
                e->kind    = KFRING_KIND_START;
                __DMB();
                kfRingWriteIdx = next_wi;
            }
        }

        imgState                       = KEYFRAME_RX;
        RAMN_SCREENIMAGE_DisplayRequested = True;

        // The START ack goes out when the drain REACHES the entry, not here.
        // It reports the previous keyframe, and the previous keyframe is not
        // finished until the drain says so -- sent from here it described a
        // frame that still had chunks queued behind it.

#ifdef SCREENIMAGE_DEBUG
        {
            char buf[80];
            int  len = snprintf(buf, sizeof(buf),
                "IMG START: %ux%u chunks=%u x=%u y=%u scale=%u\r\n",
                w, h, tc, data[6], data[7], sc);
            if (len > 0) RAMN_UART_SendFromTask((uint8_t*)buf, (uint32_t)len);
        }
#endif
        return;
    }

    // ---- 0x301: IMG_DATA (keyframe pixel chunk) ----
    if (id == IMG_CAN_ID_DATA)
    {
        if (imgState != KEYFRAME_RX)
        {
            if (kfStateDrops < 0xFFFFU) kfStateDrops++;
#ifdef SCREENIMAGE_DEBUG
            char buf[48];
            int  len = snprintf(buf, sizeof(buf),
                "IMG DATA dropped: state=%d\r\n", (int)imgState);
            if (len > 0) RAMN_UART_SendFromTask((uint8_t*)buf, (uint32_t)len);
#endif
            return;
        }
        if (dlcLen < 4U) return;   // must have at least SEQ_HI, SEQ_LO, REAL_LEN, one RLE byte

        // ---- Sequence gate ----
        // Checked before the ring is touched so a repeat costs no ring space.
        {
            uint16_t seq = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

            // Once a chunk is missing every byte after it is misaligned, so the
            // frame is over. Waiting for the next IMG_START is the only honest
            // recovery: there is nothing to resynchronise against mid-stream.
            if (kfSeqBroken != False) return;

            if (seq != kfExpectedSeq)
            {
                // Behind: the link delivered this one again. Costs nothing to
                // ignore, and re-presenting is how the SPI stage recovers a
                // response ECU D may not have taken.
                if ((uint16_t)(kfExpectedSeq - seq) <= 0x7FFFU)
                {
                    if (kfDupSkips < 0xFFFFU) kfDupSkips++;
                    return;
                }
                // Ahead: a chunk never arrived.
                kfSeqBroken = True;
                return;
            }
        }

        // Enqueue into ring buffer — writer (ReceiveCAN task) side
        uint8_t wi      = kfRingWriteIdx;
        uint8_t next_wi = (uint8_t)((wi + 1U) % KFRING_ENTRIES);
        if (next_wi == kfRingReadIdx)
        {
            // Ring full: the Periodic task has not drained fast enough. Counted
            // rather than dropped in silence -- a lost chunk desynchronises the
            // RLE stream and corrupts every pixel after it, so this number is
            // the difference between "the decoder is wrong" and "the frames
            // never got to the decoder".
            if (kfRingDrops < 0xFFFFU) kfRingDrops++;
            return;
        }

        // Payload starts at byte 3 (skip SEQ_HI, SEQ_LO, REAL_LEN). REAL_LEN
        // is the true byte count ECU D actually packed into this frame — DLC
        // is always a fixed 64 regardless of real content, so it can't be
        // used to infer this. Previously this derived payLen from dlcLen,
        // which was always 64 and therefore always computed 62, feeding
        // zero-padding into the decoder as real pixel data on every short
        // final chunk of a keyframe (bug found 2026-09-01). Both clamps below
        // are defensive: never overflow the ring entry, and never read past
        // what this specific frame actually carried even if REAL_LEN is
        // ever wrong or corrupted in transit.
        uint8_t payLen = data[2U];
        if (payLen > KFRING_PAYLOAD) payLen = KFRING_PAYLOAD;
        if (payLen > (uint8_t)(dlcLen - 3U)) payLen = (uint8_t)(dlcLen - 3U);

        KFRingEntry_t* entry = &kfRingBuf[wi];
        for (uint8_t i = 0U; i < payLen; i++)
            entry->data[i] = data[3U + i];
        entry->len  = payLen;
        entry->kind = KFRING_KIND_IMG;
        if (kfFramesRx < 0xFFFFU) kfFramesRx++;
        kfExpectedSeq++;

        __DMB();
        kfRingWriteIdx = next_wi;
        return;
    }

    // ---- 0x302: IMG_END ----
    if (id == IMG_CAN_ID_END)
    {
        // Answer even when the keyframe is not open. Staying silent here makes
        // "IMG_END never reached ECU A" and "IMG_END arrived but the keyframe
        // had already been torn down" indistinguishable on the bus, and they
        // are completely different faults.
        if (imgState != KEYFRAME_RX)
        {
            ImgFrameStats_t late;
            memset(&late, 0, sizeof late);
            SendImageAck(IMG_ACK_LATE, 0U, &late);
            return;
        }

        uint8_t status = (dlcLen >= 5U) ? data[4] : 0xFFU;

        // Queue the end behind the chunks it terminates. Answering here instead
        // read a decoder that had not run yet: with ECU D forwarding 7 chunks
        // per SPI transaction a whole burst plus its IMG_END lands inside one
        // 10 ms tick, so every keyframe was measured a fraction decoded and
        // ACKed truncated while the panel went on to receive all of it.
        uint8_t wi      = kfRingWriteIdx;
        uint8_t next_wi = (uint8_t)((wi + 1U) % KFRING_ENTRIES);
        if (next_wi != kfRingReadIdx)
        {
            // The counters travel with the entry. By the time the drain reads
            // this the CAN RX task may already be counting the NEXT keyframe,
            // so a global read there reports the wrong frame.
            KFRingEntry_t* endEntry = &kfRingBuf[wi];
            endEntry->data[0] = status;
            endEntry->data[1] = (uint8_t)(kfFramesRx   & 0xFFU);
            endEntry->data[2] = (uint8_t)(kfFramesRx   >> 8);
            endEntry->data[3] = (uint8_t)(kfRingDrops  & 0xFFU);
            endEntry->data[4] = (uint8_t)(kfRingDrops  >> 8);
            endEntry->data[5] = (uint8_t)(kfStateDrops & 0xFFU);
            endEntry->data[6] = (uint8_t)(kfStateDrops >> 8);
            endEntry->data[7] = (uint8_t)(kfDupSkips   & 0xFFU);
            endEntry->data[8] = (uint8_t)(kfDupSkips   >> 8);
            endEntry->data[9] = (kfSeqBroken != False) ? 1U : 0U;
            endEntry->len     = 10U;
            endEntry->kind    = KFRING_KIND_END;
            __DMB();
            kfRingWriteIdx = next_wi;
            return;
        }

        // Ring full: there is no room to order this behind the chunks, and an
        // unanswered IMG_END is the one outcome worth avoiding -- ECU D cannot
        // tell it apart from an ECU A that is not receiving at all. Answer now
        // and let the ring-drop flag say the numbers are understated.
        if (kfRingDrops < 0xFFFFU) kfRingDrops++;
        RAMN_Bool_t truncated = (RLE_StreamMidBlock(&kfStream) != False) ||
                                (kfSeqBroken != False);
        // Answered off the live counters: this path only runs when the ring is
        // already full, which the 0x02 flag says, so the numbers are
        // approximate by construction.
        ImgFrameStats_t st;
        st.decoded    = kfDecodedBytes;
        st.framesRx   = kfFramesRx;
        st.ringDrops  = kfRingDrops;
        st.stateDrops = kfStateDrops;
        st.dupSkips   = kfDupSkips;
        st.seqBroken  = kfSeqBroken;
        st.truncated  = truncated;
        SendImageAck(IMG_ACK_END, status, &st);

        imgState = IMG_SHOWN;

#ifdef SCREENIMAGE_DEBUG
        {
            char buf[64];
            int  len = snprintf(buf, sizeof(buf),
                "IMG END: status=%u decoded=%lu rx=%u drop=%u/%u trunc=%d\r\n",
                status, (unsigned long)kfDecodedBytes, kfFramesRx,
                kfRingDrops, kfStateDrops, (int)truncated);
            if (len > 0) RAMN_UART_SendFromTask((uint8_t*)buf, (uint32_t)len);
        }
#endif
        return;
    }

    // ---- 0x304: DELTA_FRAME_START ----
    if (id == DELTA_CAN_ID_FRAME_START)
    {
        if (imgState == IMG_IDLE) return;   // keyframe must precede delta frames
        if (dlcLen < 2U) return;

        // tileAssemblyPos used to be zeroed here. It belongs to the periodic
        // task -- the drain assembles tiles out of the ring -- and this handler
        // runs on the CAN RX task, so the write could land in the middle of a
        // tile being assembled and silently shift its rows. It was also
        // redundant: a tile's first chunk zeroes it in the drain, in order.
        // Same defect as the IMG_START reset above, one queue over.

        imgState                       = IMG_SHOWN;
        RAMN_SCREENIMAGE_DisplayRequested = True;
        return;
    }

    // ---- 0x305: DELTA_TILE_CHUNK ----
    if (id == DELTA_CAN_ID_TILE_CHUNK)
    {
        if (imgState != IMG_SHOWN)
        {
            if (kfStateDrops < 0xFFFFU) kfStateDrops++;
            return;
        }
        if (dlcLen < 5U) return;

        // Tiles carry source-space coordinates and no scale of their own, so
        // against a scaled keyframe they would land at a fraction of their
        // real position and at a fraction of their real size. The sender is
        // expected to hold scale at 1 whenever it sends deltas; if it does
        // not, refusing the tile leaves the keyframe intact where drawing it
        // would smear a wrong patch across the picture.
        if (rxScale > 1U)
        {
            if (kfTileDrops < 0xFFFFU) kfTileDrops++;
            return;
        }

        uint8_t tileX    = data[0];
        uint8_t tileY    = data[1];
        uint8_t tileSize = data[2];
        uint8_t chunkSeq = data[3];
        uint8_t payLen   = data[4];

        // Geometry is validated here and REJECTED, not clamped. Tile
        // coordinates are on an 8-pixel grid while tileSize is the extent, so a
        // 40-wide tile at tileX=29 starts at x=232 and hangs 32 pixels off the
        // panel. Clamping the window to what fits and then writing the tile's
        // bytes into it shifts every row inside the tile -- a corrupt tile drawn
        // confidently. A tile that does not fit is not a tile.
        if ((tileSize != 8U) && (tileSize != 16U) && (tileSize != 40U))
        {
            if (kfTileDrops < 0xFFFFU) kfTileDrops++;
            return;
        }
        if ((((uint16_t)tileX * 8U) + tileSize) > (uint16_t)LCD_WIDTH ||
            (((uint16_t)tileY * 8U) + tileSize) > (uint16_t)LCD_HEIGHT)
        {
            if (kfTileDrops < 0xFFFFU) kfTileDrops++;
            return;
        }
        if ((payLen == 0U) || ((uint16_t)(5U + payLen) > (uint16_t)dlcLen))
        {
            if (kfTileDrops < 0xFFFFU) kfTileDrops++;
            return;
        }
        if (payLen > KFRING_PAYLOAD) payLen = KFRING_PAYLOAD;

        // Straight into the shared ring: decoding happens in Update, where the
        // SPI writes happen, so a tile's RLE stream can be carried across chunk
        // boundaries and no tile is dropped waiting for a staging slot.
        uint8_t wi      = kfRingWriteIdx;
        uint8_t next_wi = (uint8_t)((wi + 1U) % KFRING_ENTRIES);
        if (next_wi == kfRingReadIdx)
        {
            if (kfRingDrops < 0xFFFFU) kfRingDrops++;
            return;
        }

        KFRingEntry_t* entry = &kfRingBuf[wi];
        for (uint8_t i = 0U; i < payLen; i++) entry->data[i] = data[5U + i];
        entry->len      = payLen;
        entry->kind     = KFRING_KIND_TILE;
        entry->tileX    = tileX;
        entry->tileY    = tileY;
        entry->tileSize = tileSize;
        entry->tileSeq  = chunkSeq;
        if (kfFramesRx < 0xFFFFU) kfFramesRx++;

        __DMB();
        kfRingWriteIdx = next_wi;
        return;
    }

    // ---- 0x306: DELTA_FRAME_END ----
    if (id == DELTA_CAN_ID_FRAME_END)
    {
        // All tiles for this delta frame have been written; nothing further to do.
        (void)data;
        (void)dlcLen;
        return;
    }
}

RAMNScreen_t ScreenImage = {
    .Init                = SCREENIMAGE_Init,
    .Update              = SCREENIMAGE_Update,
    .Deinit              = SCREENIMAGE_Deinit,
    .UpdateInput         = SCREENIMAGE_UpdateInput,
    .ProcessRxCANMessage = SCREENIMAGE_ProcessRxCANMessage,
};

#endif /* ENABLE_SCREEN */
