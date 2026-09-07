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
static uint16_t kfDecodedBytes = 0;

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
#define KFRING_ENTRIES   8
#define KFRING_PAYLOAD  62    // bytes per IMG_DATA frame (62 bytes of RLE payload)

typedef struct {
    uint8_t data[KFRING_PAYLOAD];
    uint8_t len;
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

// Delta tile: a fully assembled tile waiting to be written to the display
static volatile RAMN_Bool_t tileReady = False;
static uint8_t  tileStagingBuf[TILE_RAW_MAX];
static uint16_t tileStagingLen  = 0;
static uint16_t tilePendingX    = 0;
static uint16_t tilePendingY    = 0;
static uint16_t tilePendingW    = 0;
static uint16_t tilePendingH    = 0;

// Static decode buffer for keyframe ring-buffer drain — avoids large stack frame.
// Worst case: one run carried in from the previous frame (128 px = 256 bytes),
// plus 61 RLE bytes of this one = 20 repeat-runs × 128 repetitions × 2 bytes
// = 5120. 5376 total; round up.
#define KF_DECODE_BUF_SIZE  5440U
static uint8_t kfDecodeBuf[KF_DECODE_BUF_SIZE];

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
    kfRingWriteIdx  = 0;
    kfRingReadIdx   = 0;
    kfDecodedBytes  = 0;
    tileAssemblyPos = 0;
    kfWindowNeeded  = False;
    tileReady       = False;
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

            // Decode into static buffer — avoids large stack frame and ensures
            // sufficient capacity. RLE_DecodeStream carries a block that
            // straddles the frame boundary; decoding each frame on its own
            // silently loses every such block, and on a real keyframe most
            // boundaries have one.
            uint16_t decodedLen = RLE_DecodeStream(&kfStream, entry->data, entry->len,
                                                   kfDecodeBuf, KF_DECODE_BUF_SIZE);
            // An odd count is now normal, not a fault: a literal can end mid
            // pixel and its second byte arrives with the next frame. The panel
            // takes a byte stream, and the stream as a whole stays aligned.
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

    // ---- Write completed delta tile (deferred from DELTA_TILE_CHUNK handler) ----
    if (tileReady != False)
    {
        tileReady = False;
        RAMN_SPI_OpenImageWindow(tilePendingX, tilePendingY, tilePendingW, tilePendingH);
        RAMN_SPI_WriteImageChunk(tileStagingBuf, tileStagingLen);
    }

    // ---- Timeout: hold screen for IMAGE_HOLD_MS after last activity, then dismiss ----
    if (screenActive && (tick - lastActivityTick) > IMAGE_HOLD_MS)
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

        kfDecodedBytes    = 0;
        kfRingWriteIdx    = 0;
        kfRingReadIdx     = 0;
        RLE_StreamReset(&kfStream);   // a new keyframe starts a new stream
        tileAssemblyPos   = 0;
        tileReady         = False;

        imgState                       = KEYFRAME_RX;
        RAMN_SCREENIMAGE_DisplayRequested = True;

        // Defer ST7789 window open to Update() — SPI must be called from Periodic task only.
        kfPendingXOff  = kfXOffset;
        kfPendingYOff  = kfYOffset;
        kfPendingW     = kfWidth;
        kfPendingH     = kfHeight;
        kfWindowNeeded = True;

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
        if (next_wi == kfRingReadIdx) return;   // ring full — drop chunk

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
        entry->len = payLen;

        __DMB();
        kfRingWriteIdx = next_wi;
        return;
    }

    // ---- 0x302: IMG_END ----
    if (id == IMG_CAN_ID_END)
    {
        if (imgState != KEYFRAME_RX) return;

        uint8_t status = (dlcLen >= 5U) ? data[4] : 0xFFU;

        // Send ACK (0x303) back to ECU D
        FDCAN_TxHeaderTypeDef ackHdr;
        ackHdr.Identifier          = IMG_CAN_ID_ACK;
        ackHdr.IdType              = FDCAN_STANDARD_ID;
        ackHdr.TxFrameType         = FDCAN_DATA_FRAME;
        ackHdr.DataLength          = FDCAN_DLC_BYTES_2;
        ackHdr.BitRateSwitch       = FDCAN_BRS_OFF;
        ackHdr.FDFormat            = FDCAN_CLASSIC_CAN;
        ackHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        ackHdr.MessageMarker       = 0U;
        ackHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

        uint8_t ackData[2];
        ackData[0] = (status == 0x00U) ? 0x00U : 0x01U;
        ackData[1] = 0x00U;
        RAMN_FDCAN_SendMessage(&ackHdr, ackData);

        imgState = IMG_SHOWN;

#ifdef SCREENIMAGE_DEBUG
        {
            char buf[64];
            int  len = snprintf(buf, sizeof(buf),
                "IMG END: status=%u decoded=%u active=%d\r\n",
                status, kfDecodedBytes, (int)screenActive);
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
        if (imgState != IMG_SHOWN) return;
        if (dlcLen < 5U) return;

        uint8_t tileX    = data[0];
        uint8_t tileY    = data[1];
        uint8_t tileSize = data[2];
        uint8_t chunkSeq = data[3];
        uint8_t payLen   = data[4];

        // Basic validation
        if (tileX > 29U || tileY > 29U) return;
        if (tileSize != 8U && tileSize != 16U && tileSize != 40U) return;
        if (payLen == 0U || (5U + payLen) > dlcLen) return;

        // First chunk of a new tile: reset assembly buffer and record tile coords
        if ((chunkSeq & 0x7FU) == 0U)
        {
            curTileX        = tileX;
            curTileY        = tileY;
            curTileSize     = tileSize;
            tileAssemblyPos = 0U;
        }

        // RLE-decode this chunk's payload into the assembly buffer
        uint16_t decoded = RLE_Decode(&data[5], payLen,
                                       tileAssemblyBuf + tileAssemblyPos,
                                       (uint16_t)(TILE_RAW_MAX - tileAssemblyPos));
        tileAssemblyPos = (uint16_t)(tileAssemblyPos + decoded);

        // Last-chunk flag (bit 7 of chunkSeq) — stage tile for Update() to write to screen.
        if (chunkSeq & 0x80U)
        {
            uint16_t px = (uint16_t)((uint16_t)curTileX * 8U);
            uint16_t py = (uint16_t)((uint16_t)curTileY * 8U);
            uint16_t pw = (uint16_t)curTileSize;
            uint16_t ph = (uint16_t)curTileSize;

            // Clamp to screen boundaries
            if (px + pw > (uint16_t)LCD_WIDTH)  pw = (uint16_t)LCD_WIDTH  - px;
            if (py + ph > (uint16_t)LCD_HEIGHT) ph = (uint16_t)LCD_HEIGHT - py;

            uint16_t expectedBytes = (uint16_t)(pw * ph * 2U);
            if (tileAssemblyPos >= expectedBytes && (expectedBytes & 1U) == 0U)
            {
                if (tileReady == False)
                {
                    // Copy assembled tile into staging buffer and signal Update().
                    // SPI must be called from the Periodic task — defer via flag.
                    for (uint16_t i = 0U; i < expectedBytes; i++)
                        tileStagingBuf[i] = tileAssemblyBuf[i];
                    tileStagingLen = expectedBytes;
                    tilePendingX   = px;
                    tilePendingY   = py;
                    tilePendingW   = pw;
                    tilePendingH   = ph;
                    tileReady      = True;
                }
                // If tileReady is already set (Update hasn't drained it yet), drop this tile.
                // The periodic keyframe from the ESP32 will resync any missed tiles.
            }

            tileAssemblyPos = 0U;
        }
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
