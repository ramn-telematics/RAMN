#pragma once
#include "main.h"
/* Core/Inc/ramn_utils.h pulls the real main.h, which pulls the STM32 HAL.
   Only the one function ramn_screen_image.c uses is needed here; fakes.c
   provides it, mirroring the bounds guard the real one has. */
uint8_t  DLCtoUINT8(uint32_t dlc_enum);
uint32_t UINT8toDLC(uint8_t dlc);
void     RAMN_memset(void* dst, uint8_t byte, uint32_t size);
