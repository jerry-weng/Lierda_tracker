#ifndef __FLASH_H
#define __FLASH_H

#include "stm32l0xx_hal.h"

#define EEPROM_BASE_ADDR      (0x08080000)
#define EEPROM_SCHEDULE_ADDR  (0x080800FF)
#define EEPROM_END_ADDR       (0x080807FF)

extern void EEPROM_Init(void);
extern void EEPROM_WriteOneByte(uint32_t add, uint8_t data);
extern uint8_t EEPROM_ReadOneByte(uint32_t add);
extern void EEPROM_WritePage(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
extern void EEPROM_WriteBytes(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
extern void EEPROM_ReadBytes(uint32_t add, uint8_t * p_rbuf, uint32_t nbyte);

#endif
