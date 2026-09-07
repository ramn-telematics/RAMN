#pragma once
#include <stddef.h>
#include <stdint.h>

/* One 240x240 RGB565 frame is 115,200 bytes; allow slack so an over-producing
   decoder is visible as excess rather than silently clipped. */
#define FAKE_SCREEN_MAX (240 * 240 * 2 * 2)

extern uint8_t  fake_screen[FAKE_SCREEN_MAX];
extern size_t   fake_screen_len;      /* bytes written to the panel */
extern int      fake_screen_writes;   /* number of WriteImageChunk calls */
extern int      fake_screen_odd_drops; /* writes the real SPI layer would drop */
extern int      fake_window_opens;
extern uint16_t fake_window_w, fake_window_h;

/* A model of the panel itself, not just the bytes handed to it.
   RAMN_SPI_OpenImageWindow sets a rectangle and RAMN_SPI_WriteImageChunk fills
   it left to right, top to bottom, exactly as the ST7789's address-window
   auto-increment does. Without this a test can only say "3,200 bytes were
   written", not "the tile landed at x=24,y=40" -- and a tile written into the
   wrong window is precisely the delta bug that clamping geometry produced. */
#define FAKE_PANEL_W 240
#define FAKE_PANEL_H 240
extern uint8_t fake_panel[FAKE_PANEL_W * FAKE_PANEL_H * 2];
extern int     fake_panel_oob;    /* writes that ran past the current window */

void fake_screen_reset(void);
