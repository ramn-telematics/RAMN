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
static uint32_t prevDecodedBytes = 0;
static uint16_t prevFramesRx     = 0;
static uint16_t prevRingDrops    = 0;

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

// Keyframe: open a new ST7789 write window before streaming pixel data
static volatile RAMN_Bool_t kfWindowNeeded = False;
static uint8_t  kfPendingXOff = 0;
static uint8_t  kfPendingYOff = 0;
static uint16_t kfPendingW    = 0;
static uint16_t kfPendingH    = 0;

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
    kfWindowNeeded  = False;
    kfTileDrops     = 0;
    kfTileShort     = 0;
    tileActive      = False;
    RLE_StreamReset(&tileStream);
}

static void SCREENIMAGE_Update(uint32_t tick)
{
    // ---- Open keyframe ST7789 window (deferred from IMG_START handler) ----
    if (kfWindowNeeded != False)
    {
        kfWindowNeeded = False;
        RAMN_SPI_OpenImageWindow(kfPendingXOff, kfPendingYOff, kfPendingW, kfPendingH);
    }

    // ---- Drain keyframe ring buffer → RLE decode → SPI ----
    // Runs in both KEYFRAME_RX and IMG_SHOWN (to flush trailing chunks after IMG_END).
    {
        uint8_t ri = kfRingReadIdx;
        __DMB();
        while (ri != kfRingWriteIdx)
        {
            KFRingEntry_t* entry = &kfRingBuf[ri];

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
                RAMN_SPI_WriteImageChunk(kfDecodeBuf, decodedLen);
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
#define IMG_ACK_START  0x02U
#define IMG_ACK_END    0x00U
#define IMG_ACK_LATE   0x03U

static void SendImageAck(uint8_t stage, uint8_t endStatus, RAMN_Bool_t truncated)
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
    if (truncated    != False) flags |= 0x01U;
    if (kfRingDrops  != 0U)    flags |= 0x02U;
    if (kfStateDrops != 0U)    flags |= 0x04U;

    // On a START ack the current counters are all zero by definition, so report
    // the frame that just ended instead -- that is the interesting one.
    uint32_t decoded = (stage == IMG_ACK_START) ? prevDecodedBytes : kfDecodedBytes;
    uint16_t rx      = (stage == IMG_ACK_START) ? prevFramesRx     : kfFramesRx;
    uint16_t drops   = (stage == IMG_ACK_START) ? prevRingDrops    : kfRingDrops;

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

        kfWidth       = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        kfHeight      = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
        kfTotalChunks = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
        kfXOffset     = data[6];
        kfYOffset     = data[7];

        // Clamp dimensions to physical screen size
        if (kfWidth  > (uint16_t)LCD_WIDTH)  kfWidth  = (uint16_t)LCD_WIDTH;
        if (kfHeight > (uint16_t)LCD_HEIGHT) kfHeight = (uint16_t)LCD_HEIGHT;

        prevDecodedBytes  = kfDecodedBytes;
        prevFramesRx      = kfFramesRx;
        prevRingDrops     = kfRingDrops;

        kfDecodedBytes    = 0;
        kfFramesRx        = 0;
        kfRingDrops       = 0;
        kfStateDrops      = 0;
        kfRingWriteIdx    = 0;
        kfRingReadIdx     = 0;
        RLE_StreamReset(&kfStream);   // a new keyframe starts a new stream
        kfCarryValid   = False;
        tileAssemblyPos   = 0;
        tileActive        = False;
        RLE_StreamReset(&tileStream);

        imgState                       = KEYFRAME_RX;
        RAMN_SCREENIMAGE_DisplayRequested = True;

        // Defer ST7789 window open to Update() — SPI must be called from Periodic task only.
        kfPendingXOff  = kfXOffset;
        kfPendingYOff  = kfYOffset;
        kfPendingW     = kfWidth;
        kfPendingH     = kfHeight;
        kfWindowNeeded = True;

        // Say "I got IMG_START" on the bus. Without this, an ECU A that never
        // reaches IMG_END is indistinguishable from one that is not receiving
        // anything at all -- both are simply silent.
        SendImageAck(IMG_ACK_START, 0U, False);

#ifdef SCREENIMAGE_DEBUG
        {
            char buf[80];
            int  len = snprintf(buf, sizeof(buf),
                "IMG START: %ux%u chunks=%u x=%u y=%u\r\n",
                kfWidth, kfHeight, kfTotalChunks, kfXOffset, kfYOffset);
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
            SendImageAck(IMG_ACK_LATE, 0U, False);
            return;
        }

        uint8_t status = (dlcLen >= 5U) ? data[4] : 0xFFU;

        // A half-read block at IMG_END means the stream stopped mid-way -- a
        // dropped 0x301 frame, or a sender that ended early. The pixels drawn
        // so far are still valid, but the frame is incomplete, so say so in
        // the ACK rather than reporting success on a partial image.
        RAMN_Bool_t truncated = RLE_StreamMidBlock(&kfStream);
        SendImageAck(IMG_ACK_END, status, truncated);

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

        tileAssemblyPos = 0;

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
