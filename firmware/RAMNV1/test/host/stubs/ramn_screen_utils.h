#pragma once
#include "main.h"
/* The real ramn_screen_utils.h includes ramn_spi.h, and the screen modules
   rely on that rather than including it themselves. */
#include "ramn_spi.h"
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

typedef struct COLOR_THEME_STRUCT {
    uint16_t BACKGROUND;
    uint16_t DARK;
    uint16_t MEDIUM;
    uint16_t LIGHT;
    uint16_t WHITE;
} ColorTheme_t;

extern uint32_t RAMN_SCREENUTILS_LoopCounter;
extern volatile ColorTheme_t RAMN_SCREENUTILS_COLORTHEME;

/* Drawing helpers ramn_screen_regcode.c calls. Nothing is asserted about the
   pixels here -- the regcode suite's question is which frames are ACCEPTED,
   not what they look like -- so fakes_screen.c records only the calls. */
void RAMN_SCREENUTILS_DrawBase(void);
void RAMN_SCREENUTILS_DrawSubconsoleUpdate(void);
#define CONTOUR_WIDTH 2
#define LCD_WIDTH  240
#define LCD_HEIGHT 240
#define CONTROL_WINDOW_Y (LCD_HEIGHT - 34)
