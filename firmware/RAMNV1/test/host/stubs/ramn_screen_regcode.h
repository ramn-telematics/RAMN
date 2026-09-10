#pragma once
/* Stands in for Core/Inc/ramn_screen_regcode.h, which cannot be used here: it
   includes the real ramn_screen_utils.h, which pulls in the DBC, joystick and
   sensor headers and from there the HAL. Same arrangement as
   stubs/ramn_screen_image.h next door. Keep the declarations below in step
   with the real header. */
#include "main.h"
#include "ramn_screen_utils.h"

extern volatile RAMN_Bool_t RAMN_SCREENREGCODE_DisplayRequested;
#ifdef ENABLE_REGCODE_SECOC
extern volatile uint16_t RAMN_SCREENREGCODE_NoSessionDrops;
extern volatile uint16_t RAMN_SCREENREGCODE_AuthFailures;
#endif
extern RAMNScreen_t ScreenRegCode;
