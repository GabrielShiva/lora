#include "buzzer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Cálculo dos paramêtros do PWM para buzzer emitir frequência especificada
void pwm_set_frequency(uint gpio_pin, float frequency) {
    uint32_t clock   = 125000000;
    uint32_t divider = 0;
    uint32_t wrap    = 0;

    uint slice_num   = pwm_gpio_to_slice_num(gpio_pin);
    uint channel_num = pwm_gpio_to_channel(gpio_pin);

    // Se frequência for menor que zero não executa nada
    if (frequency <= 0.0f) {
        pwm_set_enabled(slice_num, false);
        return;
    }

    // Calcula os valores para o divisor e para o wrap
    divider = clock / (uint32_t)(frequency * 1000);
    wrap    = clock / (divider * (uint32_t)frequency) - 1;

    // Aplica as configurações calculados
    pwm_set_clkdiv_int_frac(slice_num, divider, 0);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, channel_num, wrap / 2); // Define o Duty cycle de 50%
}

void buzzer_setup(uint gpio_pin) {
    uint slice_num   = 0;
    uint channel_num = 0;

    // Configura o pino do buzzer para PWM e obtém as infos do pino
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    slice_num   = pwm_gpio_to_slice_num(gpio_pin);
    channel_num = pwm_gpio_to_channel(gpio_pin);

    // Configuração inicial do PWM
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);

    // Desliga PWM do pino ligado ao buzzer
    pwm_set_enabled(slice_num, false);
}

void buzzer_play(uint gpio_pin, uint frequency) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);

    pwm_set_frequency(gpio_pin, (float)frequency);
    pwm_set_enabled(slice_num, true);
}

void buzzer_stop(uint gpio_pin) {
    pwm_set_gpio_level(gpio_pin, 0);
}

