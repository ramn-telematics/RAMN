#pragma once
#include "main.h"
#define FAKE_CAN_TX_MAX 32
typedef struct { FDCAN_TxHeaderTypeDef header; uint8_t data[64]; uint8_t len; } CapturedFrame_t;
extern CapturedFrame_t fake_can_tx[FAKE_CAN_TX_MAX];
extern int             fake_can_tx_count;
extern RAMN_Result_t   fake_can_tx_result;
extern uint32_t        fake_tick;
void fake_reset(void);

extern FDCAN_ProtocolStatusTypeDef fake_can_status;
extern FDCAN_ErrorCountersTypeDef  fake_can_errors;

void fake_eeprom_reset(void);

void fake_rng_set(uint32_t seed);
