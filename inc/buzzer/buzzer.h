#ifndef BUZZER_H
#define BUZZER_H

#include <stdlib.h>
#include "pico/stdlib.h"

#define BUZZER_LEFT_PIN 21
#define BUZZER_RIGHT_PIN 10

void pwm_set_frequency(uint gpio_pin, float frequency);
void buzzer_setup(uint gpio_pin);
void buzzer_play(uint gpio_pin, uint frequency);
void buzzer_stop(uint gpio_pin);

#endif
