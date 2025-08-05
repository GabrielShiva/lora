#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Inclusão dos módulos
#include "inc/button/button.h"
#include "inc/buzzer/buzzer.h"
#include "inc/display/ssd1306.h"
#include "inc/i2c_protocol/i2c_protocol.h"
#include "inc/led_rgb/led.h"

// SPI
#define PIN_MISO 16   // SPI0 RX
#define PIN_MOSI 19   // SPI0 TX
#define PIN_SCK  18   // SPI0 SCK
#define PIN_CS   17   // Chip Select
#define PIN_RST  20   // Reset

#define SPI_PORT spi0

// Registros do SX1276
#define REG_FIFO            0x00
#define REG_OP_MODE         0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_PAYLOAD_LENGTH  0x22
#define REG_IRQ_FLAGS       0x12
#define REG_FIFO_TX_BASE    0x0E
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_MODEM_CONFIG3   0x26
#define REG_20_PREAMBLE_MSB 0x20
#define REG_21_PREAMBLE_LSB 0x21

#define MODE_SLEEP          0x00
#define MODE_STDBY          0x01
#define MODE_TX             0x03
#define MODE_RX             0x05
#define MODE_LORA           0x80
#define MODE_CAD            0x07

void write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg | 0x80, value }; // MSB = 1 para escrita
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, buf, 2);
    gpio_put(PIN_CS, 1);
}

uint8_t read_register(uint8_t reg) {
    uint8_t buf[2] = { reg & 0x7F, 0x00 }; // MSB = 0 para leitura
    uint8_t result[2];
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, buf, result, 2);
    gpio_put(PIN_CS, 1);
    return result[1];
}

void sx1276_reset() {
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);
}

void sx1276_set_frequency(uint64_t freq_hz) {
    uint64_t frf = (freq_hz << 19) / 32000000; // fórmula de conversão
    write_register(REG_FRF_MSB, (frf >> 16) & 0xFF);
    write_register(REG_FRF_MID, (frf >> 8) & 0xFF);
    write_register(REG_FRF_LSB, frf & 0xFF);
}

void sx1276_init() {
    sx1276_reset();

    // Modo LoRa e standby
    write_register(REG_OP_MODE, MODE_SLEEP | MODE_LORA);
    sleep_ms(10);

     // Ponteiro do FIFO
    write_register(REG_FIFO_TX_BASE, 0x00);
    write_register(REG_FIFO_ADDR_PTR, 0x00);
    sleep_ms(10);

    // define modo idle
    write_register(REG_OP_MODE, MODE_STDBY);
    sleep_ms(10);

    // Configurações do modem
    write_register(REG_MODEM_CONFIG1, 0x72); // BW=125kHz, CR=4/5
    write_register(REG_MODEM_CONFIG2, 0x74); // SF=7, CRC on
    write_register(REG_MODEM_CONFIG3, 0x04); // AgcAutoOn
    write_register(REG_20_PREAMBLE_MSB, 0);
    write_register(REG_21_PREAMBLE_LSB, 8);

     // Frequência 915 MHz
    sx1276_set_frequency(915000000);

    // Potência de transmissão
    write_register(REG_PA_CONFIG, 0x8F); // PA_BOOST +17 dBm
}

void sx1276_transmit(const uint8_t *data, uint8_t len) {
    write_register(REG_FIFO_ADDR_PTR, 0x00);

    for (int i = 0; i < len; i++) {
        write_register(REG_FIFO, data[i]);
    }

    write_register(REG_PAYLOAD_LENGTH, len);

    // Modo transmissão
    write_register(REG_OP_MODE, MODE_LORA | MODE_TX);

    // Espera até TX_DONE (IRQ)
    while ((read_register(REG_IRQ_FLAGS) & 0x08) == 0) {
        tight_loop_contents();
    }

    // Limpa flags de IRQ
    write_register(REG_IRQ_FLAGS, 0xFF);
}

int sx1276_receive(uint8_t *buffer, uint8_t max_len) {
    // Modo LoRa + Standby
    write_register(REG_OP_MODE, MODE_LORA | MODE_STDBY);
    write_register(REG_IRQ_FLAGS, 0xFF);  // Limpa qualquer flag anterior

    // Ponteiro FIFO de recepção
    write_register(REG_FIFO_ADDR_PTR, 0x00);

    // Ativa modo RX contínuo
    write_register(REG_OP_MODE, MODE_LORA | 0x05);  // RX_CONTINUOUS

    // Espera o pacote ser recebido (RX_DONE)
    while ((read_register(REG_IRQ_FLAGS) & 0x40) == 0) {
        tight_loop_contents();  // Espera ativamente
    }

    // Le o comprimento do payload
    uint8_t len = read_register(REG_PAYLOAD_LENGTH);
    if (len > max_len) len = max_len;

    // Ponteiro para onde o pacote foi armazenado no FIFO
    uint8_t fifo_rx_current_addr = read_register(0x10); // REG_FIFO_RX_CURRENT_ADDR
    write_register(REG_FIFO_ADDR_PTR, fifo_rx_current_addr);

    for (int i = 0; i < len; i++) {
        buffer[i] = read_register(REG_FIFO);
    }

    // Limpa flags de IRQ
    write_register(REG_IRQ_FLAGS, 0xFF);

    return len;
}

// Definição de variáveis para operação do display
static ssd1306_t ssd;
static bool color = true;
static char buffer[100];

int main() {
    stdio_init_all();

    spi_init(SPI_PORT, 5000000);
    spi_set_format(SPI_PORT, 8, 0, 0, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    // pinos do spi
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

     // Inicialização dos botões
    btns_init();

    // Inicialização do LED RGB
    leds_init();

    //Inicialização do barramento I2C para o display
    i2c_setup(I2C1_SDA, I2C1_SCL);

    printf("Inicializando o display...\n");
    // Inicializa o display
    ssd1306_setup(&ssd, WIDTH, HEIGHT, false, DISP_ADDR, I2C1_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa rádio
    sx1276_init();

    printf("Tudo pronto...\n");

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sprintf(buffer, "INICIALIZADO");
    ssd1306_draw_string(&ssd, buffer, 5, 30);
    ssd1306_send_data(&ssd);

    sleep_ms(4000);

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sleep_ms(4000);

    while (1) {
        sleep_ms(2000);

        const char *msg = "Ola LoRa";
        printf("Transmitindo: %s\n", msg);
        sx1276_transmit((const uint8_t *)msg, strlen(msg));

        printf("Transmissão finalizada.\n");

        // uint8_t buffer[64];
        // int len = sx1276_receive(buffer, sizeof(buffer));

        // buffer[len] = '\0';  // Garante término nulo para string
        // printf("Recebido (%d bytes): %s\n", len, buffer);

        // sleep_ms(500);  // Evita recepções seguidas muito rápidas
    }
}
