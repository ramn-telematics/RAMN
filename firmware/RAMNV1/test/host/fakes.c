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
RAMN_FDCAN_Status_t  RAMN_FDCAN_Status = {0, 0};

/* Bus health the tests can drive. Defaults are a healthy bus: LEC/DLEC read
   back as 7 (NO_CHANGE) on real hardware once no error has occurred since the
   last read. */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_ProtocolStatusTypeDef fake_can_status = {7, 7, 0, 0, 0, 0};
FDCAN_ErrorCountersTypeDef  fake_can_errors = {0, 0, 0};

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,
                                              FDCAN_ProtocolStatusTypeDef *s)
{ (void)h; *s = fake_can_status; return HAL_OK; }

HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h,
                                             FDCAN_ErrorCountersTypeDef *e)
{ (void)h; *e = fake_can_errors; return HAL_OK; }

void fake_reset(void) { fake_can_tx_count = 0; fake_can_tx_result = RAMN_OK;
                        RAMN_FDCAN_Status.CANRXCnt = 0; RAMN_FDCAN_Status.CANRxOverrunCnt = 0; }

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

/* ---- Emulated EEPROM ------------------------------------------------------
 * A tiny key/value store so ramn_secoc_keys.c can be exercised both
 * un-provisioned (the default-key path a freshly flashed board takes) and
 * provisioned. fake_eeprom_reset clears it back to empty. */
#include "ramn_eeprom.h"

#define FAKE_EEPROM_MAX 16
static struct { uint16_t index; uint32_t val; uint8_t used; } fake_eeprom[FAKE_EEPROM_MAX];

void fake_eeprom_reset(void)
{
	for (int i = 0; i < FAKE_EEPROM_MAX; i++) fake_eeprom[i].used = 0;
}

EE_Status RAMN_EEPROM_Init(void) { return EE_OK; }

EE_Status RAMN_EEPROM_Write32(uint16_t index, uint32_t val)
{
	for (int i = 0; i < FAKE_EEPROM_MAX; i++)
		if (fake_eeprom[i].used && fake_eeprom[i].index == index)
		{ fake_eeprom[i].val = val; return EE_OK; }
	for (int i = 0; i < FAKE_EEPROM_MAX; i++)
		if (!fake_eeprom[i].used)
		{ fake_eeprom[i].used = 1; fake_eeprom[i].index = index; fake_eeprom[i].val = val; return EE_OK; }
	return EE_WRITE_ERROR;
}

EE_Status RAMN_EEPROM_Read32(uint16_t index, uint32_t* pval)
{
	for (int i = 0; i < FAKE_EEPROM_MAX; i++)
		if (fake_eeprom[i].used && fake_eeprom[i].index == index)
		{ *pval = fake_eeprom[i].val; return EE_OK; }
	return EE_NO_DATA;
}
