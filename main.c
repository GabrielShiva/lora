#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Inclusão dos módulos
#include "inc/display/ssd1306.h"
#include "inc/i2c_protocol/i2c_protocol.h"
#include "inc/sensors/aht20.h"
#include "inc/sensors/bmp280.h"

#include "inc/sx1276.h"

// SPI
#define SPI_CHANNEL spi0 // Canal SPI
#define SPI_BAUD_RATE 5000000 // Frequência de transmissão de dados
#define SCK_PIN  18   // SPI0 SCK
#define MOSI_PIN 19   // SPI0 TX
#define MISO_PIN 16   // SPI0 RX
#define CS_PIN   17   // Seleção de dispositivo
#define RST_PIN  20   // Reset

// Definições de estrutura e variável que armazena os dados coletados pelos sensores
typedef struct sensors_data {
    int32_t pressure; // hPa * 100
    int16_t temperature; // ºC * 100
    int16_t humidity; // %RH * 100
    int32_t altitude; // m * 100
} sensors_data_t;

static sensors_data_t sensors_data = {0, 0, 0, 0};

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

// Escreve os dados na FIFO
void sx1276_write_fifo(spi_inst_t *spi, const uint8_t *data, uint8_t len) {
    uint8_t addr = REG_FIFO | 0x80; // write flag
    select_slave(CS_PIN);
    spi_write_blocking(spi, &addr, 1);
    if (len) spi_write_blocking(spi, (uint8_t*)data, len);
    unselect_slave(CS_PIN);
}

// Inicializa o chip SX1276
void sx1276_init() {
    // Reseta o dispositivo
    sx1276_reset();

    // Coloca o chip no modo SLEEP e seleciona o modo LoRa (bit 7 = 1)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_SLEEP);
    sleep_ms(1);

    // Define a frequência de 915 MHz
    sx1276_set_frequency(915000000);

    // Define o endereço do FIFO aonde os dados devem ser colocados
    // Para descrição detalhada do funcionamento, ler o item 4.1.2.3 - Principle of Operation
    // Os registradores RegFifoTxBaseAddr e RegFifoRxBaseAddr são definidos para o ponto 0x00 da memória
    // de modo que todo o espaço disponível do buffer seja utilizado (256 bytes). Por padrão, eles são
    // definidos para metade da memória (RegFifoRxBaseAddr=0x00 e RegFifoTxBaseAddr=0x80).
    write_register(SPI_CHANNEL, REG_FIFO_TX_BASE_ADDR, 0x00); // Indica o ponto na memória em que os dados que serão transmitidos estão armazenados
    write_register(SPI_CHANNEL, REG_FIFO_RX_BASE_ADDR, 0x00); // Indica a posição na memória em que os dados recebidos estarão armazenados

    // Configuração de preâmbulo igual à 8
    write_register(SPI_CHANNEL, REG_20_PREAMBLE_MSB, 0x00);
    write_register(SPI_CHANNEL, REG_21_PREAMBLE_LSB, 0x08);

    // Payload length = 10 bytes
    // write_register(SPI_CHANNEL, REG_PAYLOAD_LENGTH, 0x0A);

     /*
        RegModemConfig1 (0x1D)
        BW = 125 kHz  -> bits 7..4 = 0x07
        CR = 4/5      -> bits 3..1 = 0x01
        Header explícito -> bit 0 = 0
        => 0b01110010 = 0x72
    */
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG1, 0x72); // 0b01110010

     /*
        RegModemConfig2 (0x1E)
        SF = 7        -> bits 7..4 = 0x07
        TxContinuous = 0 -> bit 3 = 0
        CRC On        -> bit 2 = 1
        SymbTimeout bits 1..0 = 00 (default)
        => 0b01110100 = 0x74
    */
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG2, 0x74);

     /*
        RegModemConfig3 (0x26)
        LowDataRateOptimize = 0 -> bit 3 = 0
        AGC Auto On = 1         -> bit 2 = 1
        => 0b00000100 = 0x04
    */
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG3, 0x04);

    // Configura PA_BOOST para potência máxima (20 dBm)
    write_register(SPI_CHANNEL, REG_PA_CONFIG, 0xFF); // PA_BOOST + MaxPower + MaxOutputPower
    write_register(SPI_CHANNEL, REG_PA_DAC,    0x87); // High power mode (20 dBm)

    // Limite máximo do payload
    write_register(SPI_CHANNEL, 0x23 /*RegMaxPayloadLength*/, 0xFF);

    // Limpa todos os IRQs
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // O registrador RegFifoAddrPtr deve ser inicializado com o endereço do buffer em que os dados serão escritos (w) ou lidos (r).
    // Caso seja realizada a leitura, definir o valor do RegFifoRxBaseAddr. Caso seja realizada a escrita, definir
    // o valor do registrador RegFifoTxBaseAddr.
    // Neste caso, ambos os registradores foram definidos para utilizarem o espaço total disponibilizado. Sendo assim,
    // o registrador RegFifoAddrPtr deve ser inicializado com o valor 0x00 (local a partir do qual os dados recebidos ou transmitidos)
    // estão armazenados.
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00); // Indica em que endereço do FIFO deve começar a escrever os dados recebidos (TX ou RX) -> indica para qual endereço o ponteiro deve apontar
    sleep_ms(1);

    // Define modo standby
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    sleep_ms(1);
}

// Realiza a transmissão de dados
int sx1276_transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms) {
    if (len == 0) return -1;

    // Coloca o chip em modo de standby (necessário para realizar transmissão)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    sleep_ms(10);

    // Limpa todas as flags de interrupção (0b11111111)
    // Mais informações podem ser encontradas no item 4.1.2.4 do doc do SX1276
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Define a posição inicial do ponteiro do buffer do FIFO para o endereço inicial do TX (0x00)
    uint8_t tx_addr = read_register(SPI_CHANNEL, REG_FIFO_TX_BASE_ADDR);
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, tx_addr);

    // Escreve no FIFO
    sx1276_write_fifo(SPI_CHANNEL, data, len);

    // Define o tamanho do payload que será enviado (obrigatório para o modo com header implícito no registrador RegOpMode)
    write_register(SPI_CHANNEL, REG_PAYLOAD_LENGTH, len);

    // Inicia a transmissão
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_TX);

    // Espera por TxDone
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (1) {
        uint8_t irq = read_register(SPI_CHANNEL, REG_IRQ_FLAGS);
        if (irq & (1 << 3)) { // TxDone bit
            // clear IRQs
            write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);
            // go back to standby
            write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
            return 0;
        }
        // timeout
        if ((uint32_t)(to_ms_since_boot(get_absolute_time()) - start_ms) > timeout_ms) {
            // try to abort TX and clear IRQs
            write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);
            write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);
            return -1;
        }
        sleep_ms(1);
    }
}

uint8_t sx1276_receive(uint8_t *buffer, uint8_t max_len) {
    // Coloca o chip em modo de standby (necessário para realizar transmissão)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);

    // Limpa todas as flags de interrupção (0b11111111)
    // Mais informações podem ser encontradas no item 4.1.2.4 do doc do SX1276
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Define a posição inicial do ponteiro do buffer do FIFO para o endereço inicial do RX (0x00)
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00);

    // Ativa modo RX contínuo
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_RX_CONTINUOS);

     // Aguarda até que RxDone seja setado
    while ((read_register(SPI_CHANNEL, REG_IRQ_FLAGS) & 0x40) == 0) {
        tight_loop_contents(); // função do SDK para "esperar sem travar"
    }

    // Lê endereço do FIFO onde começa o pacote
    uint8_t fifo_addr = read_register(SPI_CHANNEL, REG_FIFO_RX_CURRENT_ADDR);

    // Define o ponteiro de leitura no FIFO
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, fifo_addr);

    // Lê número de bytes recebidos
    uint8_t payload_len = read_register(SPI_CHANNEL, REG_RX_NB_BYTES);

    if (payload_len > max_len) {
        payload_len = max_len; // evita overflow no buffer
    }

    // Lê os dados do FIFO
    for (int i = 0; i < payload_len; i++) {
        buffer[i] = read_register(SPI_CHANNEL, REG_FIFO);
    }

    // Limpa o flag RxDone
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0x40);

    return payload_len; // retorna o número de bytes recebidos
}

// Converte os dados do sensor de float para int
sensors_packet_t convert_to_packet(const sensors_data_t *data) {
    sensors_packet_t pkt;
    pkt.pressure = (int32_t)(data->pressure * 100);
    pkt.temperature = (int16_t)(data->temperature * 100);
    pkt.humidity = (int16_t)(data->humidity * 100);
    pkt.altitude = (int32_t)(data->altitude * 100);
    return pkt;
}

// Converte os dados de bytes para float
sensors_data_t decode_sensor_data(uint8_t *buffer) {
    sensors_data_t data;
    sensors_packet_t pkt;

    memcpy(&pkt, buffer, sizeof(pkt));

    data.pressure = pkt.pressure / 100.0f;
    data.temperature = pkt.temperature / 100.0f;
    data.humidity = pkt.humidity / 100.0f;
    data.altitude = pkt.altitude / 100.0f;

    return data;
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

    //Inicialização do barramento I2C para o display
    i2c_setup(I2C1_SDA, I2C1_SCL);

    printf("Inicializando o display...\n");
    // Inicializa o display
    ssd1306_setup(&ssd, WIDTH, HEIGHT, false, DISP_ADDR, I2C1_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sprintf(buffer, "Iniciando...");
    ssd1306_draw_string(&ssd, buffer, 5, 30);
    ssd1306_send_data(&ssd);

    // Inicializa o módulo RFM95W
    sx1276_init();

    //Inicialização do barramento I2C para os sensores
    i2c_setup(I2C0_SDA, I2C0_SCL);

    // Inicializa o BMP280
    bmp280_setup(I2C0_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C0_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C0_PORT);
    aht20_setup(I2C0_PORT);

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sprintf(buffer, "Tudo Pronto");
    ssd1306_draw_string(&ssd, buffer, 5, 30);
    ssd1306_send_data(&ssd);

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data aht20_data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure_bmp;

    char str_tmp[5];
    char str_alt[5];
    char str_umi[5];
    char str_pres[5];

    sleep_ms(2000);
    printf("Começando teste do transmissor!\n");

    const char *msg = "Ola LoRa";
    while (true) {
        printf("Enviando mensagem: %s\n", msg);

        if (sx1276_transmit((const uint8_t *)msg, strlen(msg), 5000) == 0) {
            printf("Transmissão concluída.\n");
        } else {
            printf("Falha na transmissão.\n");
        }

        sleep_ms(3000); // aguarda 3 segundos
    }

    // while (1) {
        // // Realiza leitura do AHT20
        // aht20_read(I2C0_PORT, &aht20_data);
        // sensors_data.humidity = (int32_t)(aht20_data.humidity * 100);

        // // Realiza leitura do BMP280
        // bmp280_read_raw(I2C0_PORT, &raw_temp_bmp, &raw_pressure_bmp);
        // sensors_data.temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        // sensors_data.temperature = (int16_t)(sensors_data.temperature / 100.0f * 100.0f);

        // sensors_data.pressure = bmp280_convert_pressure(raw_pressure_bmp, raw_temp_bmp, &params);
        // sensors_data.pressure = (int16_t)(sensors_data.pressure / 100.0f * 100.0f);
        // sensors_data.altitude = (int32_t)(calculate_altitude(sensors_data.pressure * 100.0) * 100.0f);

        // printf("Pressao: %.2f hPa\n", sensors_data.pressure * 100.0f);
        // printf("Temperatura: %.2f C\n", sensors_data.temperature * 100.0f);
        // printf("Altitude: %.2f m\n", sensors_data.altitude * 100.0f);
        // printf("Umidade: %.2f %%\n", sensors_data.humidity * 100.0f);

        //     // Exibe os dados no display
        //     sprintf(str_tmp, "%.1f ºC", sensors_data.temperature * 100.0f);
        //     sprintf(str_alt, "%.0f m", sensors_data.altitude * 100.0f);
        //     sprintf(str_umi, "%.1f %%", sensors_data.humidity * 100.0f);
        //     sprintf(str_pres, "%.1f hPa", sensors_data.pressure * 100.0f);

        //     //  Atualiza o conteúdo do display com animações
        //     ssd1306_fill(&ssd, !color);
        //     ssd1306_rect(&ssd, 2, 2, 124, 62, true, false);
        //     ssd1306_draw_string(&ssd, "ESTACAO", 4, 6);
        //     ssd1306_draw_string(&ssd, "LORA", 4, 14);
        //     ssd1306_line(&ssd, 3, 23, 123, 23, true); // linha horizontal - primeira
        //     ssd1306_line(&ssd, 51, 23, 51, 63, true); // linha vertical
        //     ssd1306_draw_string(&ssd, "TEMP", 4, 25);
        //     sprintf(str_tmp, "%.1fC", sensors_data.temperature);
        //     ssd1306_draw_string(&ssd, str_tmp, 54, 25);
        //     ssd1306_draw_string(&ssd, "UMID", 4, 35);
        //     sprintf(str_umi, "%.1f%%", sensors_data.humidity);
        //     ssd1306_draw_string(&ssd, str_umi, 54, 35);
        //     ssd1306_draw_string(&ssd, "ALTI", 4, 45);
        //     sprintf(str_alt, "%.1fm", sensors_data.altitude);
        //     ssd1306_draw_string(&ssd, str_alt, 54, 45);
        //     ssd1306_draw_string(&ssd, "PRES", 4, 55);
        //     sprintf(str_pres, "%.1fhPa", sensors_data.pressure);
        //     ssd1306_draw_string(&ssd, str_pres, 54, 55);
        //     ssd1306_send_data(&ssd);

        // sx1276_transmit(tx_buffer, sizeof(tx_buffer));
        // }

    return 0;
}
