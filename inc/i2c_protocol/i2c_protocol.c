#include "i2c_protocol.h"

void i2c_setup(uint8_t sda_pin, uint8_t scl_pin) {
    if (sda_pin == 0 && scl_pin == 1) {
        i2c_init(I2C0_PORT, 400 * 1000);
    }

    if (sda_pin == 14 && scl_pin == 15) {
        i2c_init(I2C1_PORT, 400 * 1000);
    }

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}
