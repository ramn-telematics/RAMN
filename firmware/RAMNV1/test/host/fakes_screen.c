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
#include "ramn_screen_utils.h"
#include "fakes_screen.h"

uint8_t  fake_panel[FAKE_PANEL_W * FAKE_PANEL_H * 2];
int      fake_panel_oob;
static uint16_t win_x, win_y, win_w, win_h;
static uint32_t win_pos;          /* pixels written into the current window */

uint8_t  fake_screen[FAKE_SCREEN_MAX];
size_t   fake_screen_len;
int      fake_screen_writes;
int      fake_screen_odd_drops;
int      fake_window_opens;
uint16_t fake_window_w, fake_window_h;
void (*fake_screen_on_write)(void);

void fake_screen_reset(void)
{
    memset(fake_screen, 0, sizeof(fake_screen));
    fake_screen_len = 0;
    fake_screen_writes = 0;
    fake_screen_odd_drops = 0;
    fake_window_opens = 0;
    fake_window_w = fake_window_h = 0;
    memset(fake_panel, 0, sizeof(fake_panel));
    fake_panel_oob = 0;
    win_x = win_y = 0;
    win_w = FAKE_PANEL_W; win_h = FAKE_PANEL_H;
    win_pos = 0;
    fake_screen_on_write = NULL;
    fake_large_chars[0] = '\0';
    fake_large_char_count = 0;
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

    /* Composite into the panel model, following the window's auto-increment. */
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        if (win_pos >= (uint32_t)win_w * win_h) { fake_panel_oob++; break; }
        uint16_t px = (uint16_t)(win_x + (win_pos % win_w));
        uint16_t py = (uint16_t)(win_y + (win_pos / win_w));
        if (px < FAKE_PANEL_W && py < FAKE_PANEL_H) {
            size_t o = ((size_t)py * FAKE_PANEL_W + px) * 2;
            fake_panel[o]     = data[i];
            fake_panel[o + 1] = data[i + 1];
        } else {
            fake_panel_oob++;
        }
        win_pos++;
    }

    /* The real call blocks here waiting on the DMA-complete notification, so
       anything the CAN RX task does lands at exactly this point. */
    if (fake_screen_on_write) fake_screen_on_write();
}

void RAMN_SPI_OpenImageWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    fake_window_opens++;
    fake_window_w = w;
    fake_window_h = h;
    win_x = x; win_y = y; win_w = w ? w : 1; win_h = h ? h : 1;
    win_pos = 0;
}

/* Inert: not the surface under test. */
void RAMN_SPI_Init(SPI_HandleTypeDef* h, void* t) { (void)h; (void)t; }
void RAMN_SPI_InitScreen(void) {}
void RAMN_SPI_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{ (void)x;(void)y;(void)w;(void)h;(void)c; }
char fake_large_chars[FAKE_LARGE_CHARS_MAX + 1];
int  fake_large_char_count;

void RAMN_SPI_DrawLargeChar(uint16_t x, uint16_t y, uint16_t f, uint16_t b, uint8_t chr, uint8_t scale)
{
    (void)x; (void)y; (void)f; (void)b; (void)scale;
    if (fake_large_char_count < FAKE_LARGE_CHARS_MAX)
        fake_large_chars[fake_large_char_count++] = (char)chr;
    fake_large_chars[fake_large_char_count] = '\0';
}

void RAMN_SPI_RefreshString(uint16_t x, uint16_t y, uint16_t f, uint16_t b, const char* s)
{ (void)x; (void)y; (void)f; (void)b; (void)s; }

volatile ColorTheme_t RAMN_SCREENUTILS_COLORTHEME;
void RAMN_SCREENUTILS_DrawBase(void) {}
void RAMN_SCREENUTILS_DrawSubconsoleUpdate(void) {}

void RAMN_SPI_DrawString(uint16_t x, uint16_t y, uint16_t f, uint16_t b, const char* s)
{ (void)x;(void)y;(void)f;(void)b;(void)s; }

uint32_t RAMN_SCREENUTILS_LoopCounter = 0;
