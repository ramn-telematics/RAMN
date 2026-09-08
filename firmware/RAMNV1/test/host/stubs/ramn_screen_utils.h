#pragma once
#include "main.h"
/* Only what ramn_screen_image.c needs to compile: the screen-module vtable and
   the joystick event type it takes. The screen manager itself is not under
   test here -- ramn_screen_image.c's own functions are called directly. */
typedef struct RAMNScreen {
    void (*Init)();
    void (*Update)();
    void (*Deinit)();
    RAMN_Bool_t (*UpdateInput)(JoystickEventType event);
    void (*ProcessRxCANMessage)();
} RAMNScreen_t;

extern uint32_t RAMN_SCREENUTILS_LoopCounter;
#define CONTOUR_WIDTH 2
#define LCD_WIDTH  240
#define LCD_HEIGHT 240
#define CONTROL_WINDOW_Y (LCD_HEIGHT - 34)
