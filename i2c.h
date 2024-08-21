#ifndef __I2C_H
#define __I2C_H

#include "stdint.h"

void i2c_setup_gpio(void);
void rcc_I2C_init(void);
void i2c_init(void);
uint8_t i2c_read(uint8_t address, uint8_t registry);
void i2c_read_many(uint8_t address, uint8_t registry, uint8_t * result, uint8_t length);
void i2c_write_many(uint8_t address, uint8_t registry, uint8_t* buf, uint32_t length);
void i2c_write(uint8_t address, uint8_t registry, uint8_t data);

#endif /* __I2C_H */

