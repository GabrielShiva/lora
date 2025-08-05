#ifndef BUTTON_H
#define BUTTON_H

#include <stdlib.h>
#include "pico/stdlib.h"

#define BTN_A_PIN 5
#define BTN_B_PIN 6
#define BTN_SW_PIN 22

void btn_setup(uint8_t gpio_pin);
void btns_init();
bool get_btn_state(uint8_t gpio_pin);

#endif
