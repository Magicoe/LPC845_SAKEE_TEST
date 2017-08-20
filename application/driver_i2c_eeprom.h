
#ifndef __DRIVER_I2C_EEPROM_H__
#define __DRIVER_I2C_EEPROM_H__

#include "board.h"
#include "stdint.h"

#define EEPROM_USE_I2C            1

#define EEPROM_I2C_PORT           LPC_I2C1
#define EEPROM_I2C_IRQHandler     I2C1_IRQHandler
#define EEPROM_I2C_IRQNUM         I2C1_IRQn

#define EEPROM_SDA_CON            IOCON_PIO1_13
#define EEPROM_SDA_PRT            1
#define EEPROM_SDA_PIN            13
#define EEPROM_SDA_NUM            0x2D

#define EEPROM_SCL_CON            IOCON_PIO1_12
#define EEPROM_SCL_PRT            1
#define EEPROM_SCL_PIN            12
#define EEPROM_SCL_NUM            0x2C

#define EEPROM_I2C_ADR            0x50

extern void eeprom_init(void);
extern void eeprom_write(uint32_t EEPROMAddr, uint8_t* EEPROMData, uint32_t EEPROMSize);
extern void eeprom_read (uint32_t EEPROMAddr, uint8_t* EEPROMData, uint32_t EEPROMSize);

extern void eeprom_test(void);

#endif  /* __DRIVER_I2C_EEPROM_H__ */
