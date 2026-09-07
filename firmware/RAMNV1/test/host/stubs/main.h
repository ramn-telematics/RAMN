/* Host-test stand-in for main.h. Provides only what ramn_telematics.c uses:
   the FDCAN types/enums, the RAMN scalar types, and the FreeRTOS surface.
   Values are copied from the real STM32L5 HAL headers -- see VALUES.md. */
#pragma once
/* The real Core/Inc/main.h includes ramn_config.h, and modules rely on that:
   ramn_screen_image.c opens with `#ifdef ENABLE_SCREEN` BEFORE including the
   config itself. Pull in the project's own config here for the same reason --
   defining ENABLE_SCREEN in the Makefile instead would let this suite build a
   configuration the firmware never does. */
#include "ramn_config.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Cortex-M data memory barrier. On the host there is one core and no store
   buffer to drain, and the compiler barrier below is enough to keep the
   ring-buffer index reads and writes in the order the source states. */
#ifndef __DMB
#define __DMB() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

typedef enum { False = 0, True = 1 } RAMN_Bool_t;
typedef enum { RAMN_OK = 0, RAMN_TRY_LATER, RAMN_ERROR } RAMN_Result_t;
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* --- FDCAN, from stm32l5xx_hal_fdcan.h --------------------------------- */
#define FDCAN_STANDARD_ID       ((uint32_t)0x00000000U)
#define FDCAN_EXTENDED_ID       ((uint32_t)0x40000000U)
#define FDCAN_DATA_FRAME        ((uint32_t)0x00000000U)
#define FDCAN_REMOTE_FRAME      ((uint32_t)0x20000000U)
#define FDCAN_ESI_ACTIVE        ((uint32_t)0x00000000U)
#define FDCAN_BRS_OFF           ((uint32_t)0x00000000U)
#define FDCAN_BRS_ON            ((uint32_t)0x00100000U)
#define FDCAN_CLASSIC_CAN       ((uint32_t)0x00000000U)
#define FDCAN_FD_CAN            ((uint32_t)0x00200000U)
#define FDCAN_NO_TX_EVENTS      ((uint32_t)0x00000000U)
#define FDCAN_DLC_BYTES_0       ((uint32_t)0x00000000U)
#define FDCAN_DLC_BYTES_4       ((uint32_t)0x00000004U)
#define FDCAN_DLC_BYTES_8       ((uint32_t)0x00000008U)
#define FDCAN_DLC_BYTES_12      ((uint32_t)0x00000009U)
#define FDCAN_DLC_BYTES_1       ((uint32_t)0x00000001U)
#define FDCAN_DLC_BYTES_2       ((uint32_t)0x00000002U)
#define FDCAN_DLC_BYTES_3       ((uint32_t)0x00000003U)
#define FDCAN_DLC_BYTES_5       ((uint32_t)0x00000005U)
#define FDCAN_DLC_BYTES_6       ((uint32_t)0x00000006U)
#define FDCAN_DLC_BYTES_7       ((uint32_t)0x00000007U)
#define FDCAN_DLC_BYTES_16      ((uint32_t)0x0000000AU)
#define FDCAN_DLC_BYTES_20      ((uint32_t)0x0000000BU)
#define FDCAN_DLC_BYTES_24      ((uint32_t)0x0000000CU)
#define FDCAN_DLC_BYTES_32      ((uint32_t)0x0000000DU)
#define FDCAN_DLC_BYTES_48      ((uint32_t)0x0000000EU)
#define FDCAN_DLC_BYTES_64      ((uint32_t)0x0000000FU)

/* Joystick events, as the real main.h declares them. ramn_screen_image.c's
   UpdateInput takes them; only the two it tests against are used. */
typedef enum {
    JOYSTICK_EVENT_NONE = 0,
    JOYSTICK_EVENT_LEFT_PRESSED,
    JOYSTICK_EVENT_RIGHT_PRESSED,
    JOYSTICK_EVENT_LEFT_RELEASED,
    JOYSTICK_EVENT_RIGHT_RELEASED,
    JOYSTICK_EVENT_UP_PRESSED,
    JOYSTICK_EVENT_DOWN_PRESSED,
    JOYSTICK_EVENT_PRESS
} JoystickEventType;

typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator,
             BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, ErrorStateIndicator,
             BitRateSwitch, FDFormat, RxTimestamp, FilterIndex, IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

/* --- SPI / GPIO -------------------------------------------------------- */
typedef struct { int dummy; } SPI_HandleTypeDef;
typedef struct { int dummy; } GPIO_TypeDef;
#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0
#define LCD_nCS_GPIO_Port ((GPIO_TypeDef*)0)
#define LCD_nCS_Pin       0
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef*);

/* --- FreeRTOS ---------------------------------------------------------- */
typedef void* StreamBufferHandle_t;
uint32_t xTaskGetTickCount(void);
void taskENTER_CRITICAL(void);
void taskEXIT_CRITICAL(void);
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t);
size_t xStreamBufferSpacesAvailable(StreamBufferHandle_t);
#define pdMS_TO_TICKS(x) (x)

/* --- ramn_utils --- */
uint8_t  DLCtoUINT8(uint32_t dlc_enum);
uint32_t UINT8toDLC(uint8_t dlc);
void     RAMN_memset(void* dst, uint8_t byte, uint32_t size);
