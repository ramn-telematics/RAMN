#pragma once
#include "main.h"
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
/* Only what ramn_screen_image.c calls. The screen is the assertion surface:
   fakes_screen.c records every RAMN_SPI_WriteImageChunk, which is exactly the
   pixel data that would reach the ST7789. */
void RAMN_SPI_Init(SPI_HandleTypeDef* handler, void* pTask);
void RAMN_SPI_InitScreen(void);
void RAMN_SPI_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void RAMN_SPI_DrawString(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, const char* src);
void RAMN_SPI_OpenImageWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void RAMN_SPI_WriteImageChunk(const uint8_t* data, uint16_t len);
