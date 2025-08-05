#include "button.h"

void btn_setup(uint8_t gpio_pin) {
    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_IN);
    gpio_pull_up(gpio_pin);
}

void btns_init() {
    btn_setup(BTN_A_PIN);
    btn_setup(BTN_B_PIN);
    btn_setup(BTN_SW_PIN);
}

bool get_btn_state(uint8_t gpio_pin) {
    return gpio_get(gpio_pin) == 1 ? true : false;
}
