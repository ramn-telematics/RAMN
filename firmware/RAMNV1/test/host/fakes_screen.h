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

void fake_screen_reset(void);
