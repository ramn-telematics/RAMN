#pragma once
#include "main.h"
void RAMN_UART_SendFromTask(uint8_t* data, uint32_t size);
void RAMN_UART_SendStringFromTask(const char* str);
