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

#include "inc/sx1276.h"

// SPI
#define SPI_CHANNEL spi0 // Canal SPI
#define SPI_BAUD_RATE 5000000 // Frequência de transmissão de dados
#define SCK_PIN  18   // SPI0 SCK
#define MOSI_PIN 19   // SPI0 TX
#define MISO_PIN 16   // SPI0 RX
#define CS_PIN   17   // Seleção de dispositivo
#define RST_PIN  20   // Reset

// Seleciona o dispositivo de comunicação no barramento SPI
void select_slave(uint chip_select) {
    gpio_put(chip_select, 0);
}

// Desceleciona o dispositivo de comunicação no barramento SPI
void unselect_slave(uint chip_select) {
    gpio_put(chip_select, 1);
}

// Escreve no registrador com endereço ADDRESS o valor VALUE
void write_register(spi_inst_t *spi_channel, uint8_t address, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = address | 0x80;
    buffer[1] = value;

    select_slave(CS_PIN);
    spi_write_blocking(spi_channel, buffer, 2);
    unselect_slave(CS_PIN);
}

// Realiza a leitura do registrador com endereço ADDRESS
uint8_t read_register(spi_inst_t *spi_channel, uint8_t address) {
    uint8_t buffer[2];
    buffer[0] = address & 0x7F;
    buffer[1] = 0x00;

    // Armazena o valor lido
    uint8_t result[2];

    select_slave(CS_PIN);
    spi_write_read_blocking(spi_channel, buffer, result, 2);
    unselect_slave(CS_PIN);

    // Retorna o valor lido
    return result[1];
}

// Reseta o chip SX1276
void sx1276_reset() {
    gpio_put(RST_PIN, 0);
    sleep_ms(100);
    gpio_put(RST_PIN, 1);
    sleep_ms(100);
}

// Define a frequência de operação do chip (sinal)
void sx1276_set_frequency(uint64_t frequency_hz) {
    // Realiza a conversão do valor especificado para um número de 24 bits
    uint64_t new_freq = (frequency_hz << 19) / 32000000;

    // Escreve o valor especificado nos registradores
    write_register(SPI_CHANNEL, REG_FRF_MSB, (uint8_t)(new_freq >> 16));
    write_register(SPI_CHANNEL, REG_FRF_MID, (uint8_t)(new_freq >> 8));
    write_register(SPI_CHANNEL, REG_FRF_LSB, (uint8_t)(new_freq >> 0));
}

// Inicializa o chip SX1276
void sx1276_init() {
    // Reseta o dispositivo
    sx1276_reset();

    // Verifica a versão do chip
    uint8_t version = read_register(SPI_CHANNEL, REG_VERSION);
    if (version != 0x12) {
        printf("O chip SX1276 nao foi encontrado! Versao: 0x%02X\n", version);
    } else {
        printf("SX1276 OK. Versao: 0x%02X\n", version);
    }

    // Coloca o chip no modo SLEEP e seleciona o modo LoRa (bit 7 = 1)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_SLEEP);
    sleep_ms(10);

    // Define a frequência de 915 MHz
    sx1276_set_frequency(915000000);

    // Configurações do modem
    // BW=125 kHz, CR=4/5, Header explícito (ImplicitHeaderMode = 0 -> não é necessário indicar o tamanho do payload no registrador RegPayloadLength)
    // SF=7 (0111), TxContinuousMode=0 (envio de um pacote apenas), RxPayloadCrcOn=1, SymbTimeout(9:8)=0x00
    // 0000 (não representa nada), LowDataRateOptimize=0 (desabilitado), AgcAutoOn=1, 00 (reservado)
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG1, 0x72); // 0b01110010
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG2, 0x74); // 0b01110100
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG3, 0x04); // 0b00000100

    // Configuração de preâmbulo igual à 8
    write_register(SPI_CHANNEL, REG_20_PREAMBLE_MSB, 0);
    write_register(SPI_CHANNEL, REG_21_PREAMBLE_LSB, 8);

    // Define a potência (17 dBm)
    write_register(SPI_CHANNEL, REG_PA_CONFIG, 0x8F);
    sleep_ms(10);

    // Define o endereço do FIFO aonde os dados devem ser colocados
    // Para descrição detalhada do funcionamento, ler o item 4.1.2.3 - Principle of Operation
    // Os registradores RegFifoTxBaseAddr e RegFifoRxBaseAddr são definidos para o ponto 0x00 da memória
    // de modo que todo o espaço disponível do buffer seja utilizado (256 bytes). Por padrão, eles são
    // definidos para metade da memória (RegFifoRxBaseAddr=0x00 e RegFifoTxBaseAddr=0x80).
    write_register(SPI_CHANNEL, REG_FIFO_TX_BASE_ADDR, 0x00); // Indica o ponto na memória em que os dados que serão transmitidos estão armazenados
    write_register(SPI_CHANNEL, REG_FIFO_RX_BASE_ADDR, 0x00); // Indica a posição na memória em que os dados recebidos estarão armazenados
    // O registrador RegFifoAddrPtr deve ser inicializado com o endereço do buffer em que os dados serão escritos (w) ou lidos (r).
    // Caso seja realizada a leitura, definir o valor do RegFifoRxBaseAddr. Caso seja realizada a escrita, definir
    // o valor do registrador RegFifoTxBaseAddr.
    // Neste caso, ambos os registradores foram definidos para utilizarem o espaço total disponibilizado. Sendo assim,
    // o registrador RegFifoAddrPtr deve ser inicializado com o valor 0x00 (local a partir do qual os dados recebidos ou transmitidos)
    // estão armazenados.
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00); // Indica em que endereço do FIFO deve começar a escrever os dados recebidos (TX ou RX) -> indica para qual endereço o ponteiro deve apontar
    sleep_ms(10);

    // define modo standby
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_STDBY);
    sleep_ms(10);
}

// Realiza a transmissão de dados
void sx1276_transmit(uint8_t *data, uint8_t len) {
    // Coloca o chip em modo de standby (necessário para realizar transmissão)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    sleep_ms(10);

    // Define a posição inicial do ponteiro do buffer do FIFO para o endereço inicial do TX (0x00)
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00);

    // Escreve os dados no buffer do FIFO
    for (int i = 0; i < len; i++) {
        write_register(SPI_CHANNEL, REG_FIFO, data[i]);
    }

    // Define o tamanho do payload que será enviado
    write_register(SPI_CHANNEL, REG_PAYLOAD_LENGTH, len);

    // Limpa as flags de interrupção
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Define o modo de transmissão (TX)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_TX);

    // Espera até o bit TX_DONE do registrador RegIrqFlags ser setado
    while ((read_register(SPI_CHANNEL, REG_IRQ_FLAGS) & 0x08) == 0) {
        tight_loop_contents();
    }

    // Limpa as flags de interrupção
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Define o modo de standby
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);

    printf("Transmissão de dados realizada.\n");
}

int sx1276_receive(uint8_t *buffer, uint8_t max_len) {
    // Modo LoRa + Standby
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);  // Limpa qualquer flag anterior

    // Ponteiro FIFO de recepção
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00);

    // Ativa modo RX contínuo
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | 0x05);  // RX_CONTINUOUS

    // Espera o pacote ser recebido (RX_DONE)
    while ((read_register(SPI_CHANNEL, REG_IRQ_FLAGS) & 0x40) == 0) {
        tight_loop_contents();  // Espera ativamente
    }

    // Le o comprimento do payload
    uint8_t len = read_register(SPI_CHANNEL, REG_PAYLOAD_LENGTH);
    if (len > max_len) len = max_len;

    // Ponteiro para onde o pacote foi armazenado no FIFO
    uint8_t fifo_rx_current_addr = read_register(SPI_CHANNEL, 0x10); // REG_FIFO_RX_CURRENT_ADDR
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, fifo_rx_current_addr);

    for (int i = 0; i < len; i++) {
        buffer[i] = read_register(SPI_CHANNEL, REG_FIFO);
    }

    // Limpa flags de IRQ
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    return len;
}

// Definição de variáveis para operação do display
static ssd1306_t ssd;
static bool color = true;
static char buffer[100];

int main() {
    stdio_init_all();

    // Configuração da interface SPI
    spi_init(SPI_CHANNEL, SPI_BAUD_RATE);
    spi_set_format(SPI_CHANNEL, 8, 0, 0, 0);

    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    // Pinos de seleção SPI e de reset para o módulo RFM95W
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    gpio_init(RST_PIN);
    gpio_set_dir(RST_PIN, GPIO_OUT);

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

    // Inicializa o módulo RFM95W
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
        uint8_t msg[] = "Hello, World!";

        sx1276_transmit(&msg, sizeof(msg) - 1);

        sleep_ms(3000);

        // uint8_t buffer[64];
        // int len = sx1276_receive(buffer, sizeof(buffer));

        // buffer[len] = '\0';  // Garante término nulo para string
        // printf("Recebido (%d bytes): %s\n", len, buffer);

        // sleep_ms(500);  // Evita recepções seguidas muito rápidas
    }
}
