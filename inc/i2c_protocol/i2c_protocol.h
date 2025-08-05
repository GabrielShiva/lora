#ifndef I2C_PROTOCOL_H
#define I2C_PROTOCOL_H

#include <stdlib.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#define I2C0_PORT i2c0
#define I2C0_SDA 0
#define I2C0_SCL 1

#define I2C1_PORT i2c1
#define I2C1_SDA 14
#define I2C1_SCL 15

void i2c_setup(uint8_t sda_pin, uint8_t scl_pin);

#endif
