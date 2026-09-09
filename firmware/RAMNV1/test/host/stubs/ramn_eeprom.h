/* Host-test stand-in for ramn_eeprom.h.
 *
 * The real header pulls in ST's eeprom_emul.h, which lives under Middlewares
 * and needs the HAL. The only thing the code under test wants from it is the
 * 32-bit read/write pair and EE_OK, so that is all this provides. The fake
 * implementation is in fakes.c: it defaults to "nothing stored", which drives
 * ramn_secoc_keys.c down its default-key path -- the state a freshly flashed
 * board is actually in. */
#pragma once
#include <stdint.h>

typedef enum { EE_OK = 0, EE_NO_DATA = 1, EE_WRITE_ERROR = 2 } EE_Status;

EE_Status RAMN_EEPROM_Init(void);
EE_Status RAMN_EEPROM_Write32(uint16_t index, uint32_t val);
EE_Status RAMN_EEPROM_Read32(uint16_t index, uint32_t* pval);
