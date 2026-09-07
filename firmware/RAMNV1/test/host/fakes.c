/* Host-side fakes. RAMN_FDCAN_SendMessage records every frame the code under
   test tries to put on the bus -- that recording IS the assertion surface. */
#include "main.h"
#include "ramn_canfd.h"
#include "ramn_uart.h"
#include <stdio.h>
#include "fakes.h"

CapturedFrame_t fake_can_tx[FAKE_CAN_TX_MAX];
int             fake_can_tx_count = 0;
RAMN_Result_t   fake_can_tx_result = RAMN_OK;
uint32_t        fake_tick = 0;

StreamBufferHandle_t CANTxDataStreamBufferHandle = (StreamBufferHandle_t)1;

void fake_reset(void) { fake_can_tx_count = 0; fake_can_tx_result = RAMN_OK; }

RAMN_Result_t RAMN_FDCAN_SendMessage(const FDCAN_TxHeaderTypeDef* h, const uint8_t* data)
{
    if (fake_can_tx_result != RAMN_OK) return fake_can_tx_result;
    if (fake_can_tx_count >= FAKE_CAN_TX_MAX) return RAMN_ERROR;

    CapturedFrame_t *c = &fake_can_tx[fake_can_tx_count++];
    c->header = *h;
    /* Mirror what the real send path does: the payload length comes from
       DLCtoUINT8(DataLength), and a remote frame carries none. */
    uint8_t dlc = DLCtoUINT8(h->DataLength);
    if (h->TxFrameType == FDCAN_REMOTE_FRAME) dlc = 0U;
    c->len = dlc;
    memcpy(c->data, data, dlc);
    return RAMN_OK;
}

/* ramn_utils, copied verbatim from ramn_utils.c */
static const uint8_t DlcToUint8convTable[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
/* Mirrors the bounds guard in ramn_utils.c. A fake that is more permissive
   than the real thing hides the bug it is standing in for. */
uint8_t  DLCtoUINT8(uint32_t e) { return (e > 15U) ? 0U : DlcToUint8convTable[(uint8_t)e]; }
uint32_t UINT8toDLC(uint8_t dlc) { return dlc; }
void RAMN_memset(void* dst, uint8_t b, uint32_t n) { memset(dst, b, n); }

/* Inert peripheral surface */
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, int s) { (void)p;(void)pin;(void)s; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*a,uint8_t*b,uint8_t*c,uint16_t d)
{ (void)a;(void)b;(void)c;(void)d; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*a,uint8_t*b,uint16_t c)
{ (void)a;(void)b;(void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef*a) { (void)a; return HAL_OK; }
uint32_t xTaskGetTickCount(void) { return fake_tick; }
void taskENTER_CRITICAL(void) {}
void taskEXIT_CRITICAL(void) {}
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t h) { (void)h; return 0; }
size_t xStreamBufferSpacesAvailable(StreamBufferHandle_t h) { (void)h; return 4096; }
void RAMN_UART_SendFromTask(uint8_t* d, uint32_t n) { (void)d;(void)n; }
void RAMN_UART_SendStringFromTask(const char* s) { (void)s; }
SPI_HandleTypeDef hspi2;
