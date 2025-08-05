#include "led.h"

void led_setup(uint8_t gpio_pin) {
    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_OUT);
    gpio_put(gpio_pin, 0);
}

void leds_init() {
    led_setup(GREEN_LED_PIN);
    led_setup(BLUE_LED_PIN);
    led_setup(RED_LED_PIN);
}

void set_led_off(uint8_t gpio_pin) {
    gpio_put(gpio_pin, 0);
}

void set_led_on(uint8_t gpio_pin) {
    gpio_put(gpio_pin, 1);
}

void leds_turnoff() {
    set_led_off(GREEN_LED_PIN);
    set_led_off(RED_LED_PIN);
    set_led_off(BLUE_LED_PIN);
}

void leds_turnon() {
    set_led_on(GREEN_LED_PIN);
    set_led_on(RED_LED_PIN);
    set_led_on(BLUE_LED_PIN);
}

void set_led_green() {
    set_led_on(GREEN_LED_PIN);
}

void set_led_blue() {
    set_led_on(BLUE_LED_PIN);
}

void set_led_red() {
    set_led_on(RED_LED_PIN);
}
