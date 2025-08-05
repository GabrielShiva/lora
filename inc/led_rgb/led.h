#ifndef LED_H
#define LED_H

#include <stdlib.h>
#include "pico/stdlib.h"

#define GREEN_LED_PIN 11
#define BLUE_LED_PIN 12
#define RED_LED_PIN 13

void led_setup(uint8_t gpio_pin);
void leds_init();

void set_led_off(uint8_t gpio_pin);
void set_led_on(uint8_t gpio_pin);

void leds_turnon();
void leds_turnoff();

void set_led_green();
void set_led_blue();
void set_led_red();

#endif
