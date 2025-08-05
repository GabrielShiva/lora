#ifndef MPU6050_H
#define MPU6050_H

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDR 0x68

void mpu6050_reset(i2c_inst_t *i2c);
void mpu6050_read_raw(i2c_inst_t *i2c, int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif
