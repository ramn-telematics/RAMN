/* Fakes for the ECU A image screen.
 *
 * The assertion surface is RAMN_SPI_WriteImageChunk: exactly the pixel bytes
 * that would reach the ST7789. Every test states what SHOULD land on the
 * screen for a given set of CAN frames, which is the only question that
 * matters about a decoder.
 */
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "fakes_screen.h"

uint8_t  fake_screen[FAKE_SCREEN_MAX];
size_t   fake_screen_len;
int      fake_screen_writes;
int      fake_screen_odd_drops;
int      fake_window_opens;
uint16_t fake_window_w, fake_window_h;

void fake_screen_reset(void)
{
    memset(fake_screen, 0, sizeof(fake_screen));
    fake_screen_len = 0;
    fake_screen_writes = 0;
    fake_screen_odd_drops = 0;
    fake_window_opens = 0;
    fake_window_w = fake_window_h = 0;
}

void RAMN_SPI_WriteImageChunk(const uint8_t* data, uint16_t len)
{
    /* Mirrors the real RAMN_SPI_WriteImageChunk in ramn_spi.c, which drops
       zero-length and ODD-length writes on the floor and returns. A fake more
       permissive than the thing it stands in for hides exactly the bug it
       exists to catch -- this one hid a blank screen. */
    if (len == 0U || (len & 1U)) { fake_screen_odd_drops++; return; }
    fake_screen_writes++;
    if (fake_screen_len + len > FAKE_SCREEN_MAX) len = (uint16_t)(FAKE_SCREEN_MAX - fake_screen_len);
    memcpy(&fake_screen[fake_screen_len], data, len);
    fake_screen_len += len;
}

void RAMN_SPI_OpenImageWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    (void)x; (void)y;
    fake_window_opens++;
    fake_window_w = w;
    fake_window_h = h;
}

/* Inert: not the surface under test. */
void RAMN_SPI_Init(SPI_HandleTypeDef* h, void* t) { (void)h; (void)t; }
void RAMN_SPI_InitScreen(void) {}
void RAMN_SPI_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{ (void)x;(void)y;(void)w;(void)h;(void)c; }
void RAMN_SPI_DrawString(uint16_t x, uint16_t y, uint16_t f, uint16_t b, const char* s)
{ (void)x;(void)y;(void)f;(void)b;(void)s; }

uint32_t RAMN_SCREENUTILS_LoopCounter = 0;
