#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Inclusão dos módulos
#include "inc/button/button.h"
#include "inc/buzzer/buzzer.h"
#include "inc/display/ssd1306.h"
#include "inc/i2c_protocol/i2c_protocol.h"
#include "inc/led_rgb/led.h"
#include "inc/sensors/aht20.h"
#include "inc/sensors/bmp280.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <math.h>

#include "inc/sx1276.h"

// SPI
#define SPI_CHANNEL spi0 // Canal SPI
#define SPI_BAUD_RATE 5000000 // Frequência de transmissão de dados
#define SCK_PIN  18   // SPI0 SCK
#define MOSI_PIN 19   // SPI0 TX
#define MISO_PIN 16   // SPI0 RX
#define CS_PIN   17   // Seleção de dispositivo
#define RST_PIN  20   // Reset

// Definição de variáveis e macros importantes para o debounce dos botões
#define DEBOUNCE_TIME 260

static volatile uint32_t last_btn_a_press = 0;
static volatile uint32_t last_btn_b_press = 0;

// Define a página que é exibida no display
static volatile bool display_page = 0;

// Definições de estrutura e variável que armazena os dados coletados pelos sensores
typedef struct sensors_data {
    float pressure;
    float temperature;
    float humidity;
    float altitude;
} sensors_data_t;

static sensors_data_t sensors_data = {0.0f, 0.0f, 0.0f, 0.0f};

typedef struct {
    int32_t pressure;     // in hPa × 100
    int16_t temperature;  // in °C × 100
    int16_t humidity;     // in %RH × 100
    int32_t altitude;     // in meters × 100
} sensors_packet_t;

QueueHandle_t xBMPReadQueue;
QueueHandle_t xAHTReadQueue;

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
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG1, 0x72); // 0b01110011
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG2, 0x77);
    write_register(SPI_CHANNEL, REG_MODEM_CONFIG3, 0x04); // 0b00000100

    // Configuração de preâmbulo igual à 8
    write_register(SPI_CHANNEL, REG_20_PREAMBLE_MSB, 0);
    write_register(SPI_CHANNEL, REG_21_PREAMBLE_LSB, 8);

    // Define a potência (20 dBm)
    write_register(SPI_CHANNEL, REG_PA_CONFIG, 0x8F); // PA_BOOST, max power
    write_register(SPI_CHANNEL, REG_PA_DAC, 0x87);
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
    vTaskDelay(pdMS_TO_TICKS(10));

    // Define a posição inicial do ponteiro do buffer do FIFO para o endereço inicial do TX (0x00)
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, 0x00);

    // Define o tamanho do payload que será enviado (obrigatório para o modo com header implícito no registrador RegOpMode)
    write_register(SPI_CHANNEL, REG_PAYLOAD_LENGTH, len);

    // Escreve os dados no buffer do FIFO
    for (uint8_t i = 0; i < len; i++) {
        write_register(SPI_CHANNEL, REG_FIFO, data[i]);
    }

    // Limpa todas as flags de interrupção (0b11111111)
    // Mais informações podem ser encontradas no item 4.1.2.4 do doc do SX1276
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Define o modo de transmissão (TX)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_TX);

    // Verifica por meio do pooling se a transmissão terminou (bit TX_DONE do registrador RegIrqFlags será setado)
    while ((read_register(SPI_CHANNEL, REG_IRQ_FLAGS) & 0x08) == 0) {
        tight_loop_contents();
    }

    // Ao finalizar o envio dos pacotes de dados o dispositivo entre em modo standby automáticamente (pode ser visto em Data Transmission Sequence)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);

    // Limpa as flags de interrupção
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    printf("Transmissão de dados realizada.\n");
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
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | 0x05);  // RX_CONTINUOUS

    // Espera o pacote ser recebido (RX_DONE)
    while ((read_register(SPI_CHANNEL, REG_IRQ_FLAGS) & (1 << 6)) == 0) {
        tight_loop_contents();
    }

    // Verificar se ocorreu erro (CRC) -> Flag PayloadCrcError
    uint8_t irq_flags = read_register(SPI_CHANNEL, REG_IRQ_FLAGS);
    if (irq_flags & (1 << 5)) {
        write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);
        printf("Erro em pacote recebido");
        return 0;
    }

    // Le o comprimento do payload
    uint8_t num_bytes_pckt = (uint8_t)read_register(SPI_CHANNEL, REG_RX_NB_BYTES);
    if (num_bytes_pckt > max_len) {
        num_bytes_pckt = max_len;
    }

    // Obtém o endereço do local da memória em que o último pacote foi armazenado no FIFO
    // Coloca o ponteiro no local da memória onde o último pacote foi armazenado no FIFO
    uint8_t current_addr = read_register(SPI_CHANNEL, REG_FIFO_RX_CURRENT_ADDR);
    write_register(SPI_CHANNEL, REG_FIFO_ADDR_PTR, current_addr);

    // Armazena o pacote recebido na variável buffer
    for (uint8_t i = 0; i < num_bytes_pckt; i++) {
        buffer[i] = read_register(SPI_CHANNEL, REG_FIFO);
    }

    // Limpa as flags de IRQ
    write_register(SPI_CHANNEL, REG_IRQ_FLAGS, 0xFF);

    // Retorna ao modo standby (em modo RX contínuo ele não volta automáticamente)
    write_register(SPI_CHANNEL, REG_OP_MODE, MODE_LORA | MODE_STDBY);

    printf("Foram recebidos %d bytes. \n", num_bytes_pckt);

    return num_bytes_pckt;
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

// Função responsável por realizar o tratamento das interrupções geradas pelos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Muda a página exibida no display
    if (gpio == BTN_B_PIN && (current_time - last_btn_b_press > DEBOUNCE_TIME)) {
        last_btn_b_press = current_time;
        reset_usb_boot(0, 0);
    }
}

// Definição de variáveis para operação do display
static ssd1306_t ssd;
static bool color = true;
static char buffer[100];

void vLoraTask();
void vSensorRead();

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

    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

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

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sprintf(buffer, "Iniciando...");
    ssd1306_draw_string(&ssd, buffer, 5, 30);
    ssd1306_send_data(&ssd);

    // Inicializa o módulo RFM95W
    sx1276_init();

    //Inicialização do barramento I2C para os sensores
    i2c_setup(I2C0_SDA, I2C0_SCL);

    printf("Tudo pronto...\n");

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sprintf(buffer, "Tudo Pronto");
    ssd1306_draw_string(&ssd, buffer, 5, 30);
    ssd1306_send_data(&ssd);

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    sleep_ms(2000);
    printf("Começando teste do transmissor!\n");

    xAHTReadQueue = xQueueCreate(1, sizeof(AHT20_Data));
    xBMPReadQueue = xQueueCreate(1, sizeof(BMP280_Data));

    //xTaskCreate(vHelloTask, "Hello Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vSensorRead, "Sensor Read", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLoraTask, "Connect task", 2048, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
    panic_unsupported();

}

void vLoraTask(){
    // Transmissor

    AHT20_Data aht_data;
    BMP280_Data bmp_data;

    char message[200];
    uint32_t counter = 0;
    //printf("tarefa lora");

    while (true)
    {
        xQueuePeek(xAHTReadQueue, &aht_data, 0);
        xQueuePeek(xBMPReadQueue, &bmp_data, 0);
        memset(message, 0, sizeof(message));
        sprintf(message, "Pacote #%d - Hora: %d ms", counter++, time_us_32() / 1000);
        sx1276_transmit((uint8_t*)message, strlen(message));
        printf(message);
        printf("\n");
        memset(message, 0, sizeof(message));
        sprintf(message, "%d - Umidade: %.2f percent", counter, aht_data.humidity);
        sx1276_transmit((uint8_t*)message, strlen(message));
        printf(message);
        printf("\n");
        sprintf(message, "%d - Temperatura: %.2f C", counter, aht_data.temperature);
        sx1276_transmit((uint8_t*)message, strlen(message));
        printf(message);
        printf("\n");
        sprintf(message, "%d - Altitude (n. do mar): %.2fm", counter, bmp_data.altitude);
        sx1276_transmit((uint8_t*)message, strlen(message));
        printf(message);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void vSensorRead(){
    // Inicializa o BMP280
    bmp280_setup(I2C0_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C0_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C0_PORT);
    aht20_setup(I2C0_PORT);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data aht_data;
    BMP280_Data bmp_data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    while(true){
        // Leitura do BMP280
        bmp280_read_raw(I2C0_PORT, &raw_temp_bmp, &raw_pressure);
        bmp_data.temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        bmp_data.pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        bmp_data.altitude = calculate_altitude(bmp_data.pressure);
        //printf("tarefa do sensor!\n");
        //printf("Pressao = %.3f kPa\n", bmp_data.pressure / 1000.0);
        //printf("Temperatura BMP: = %.2f C\n", bmp_data.temperature / 100.0);
        //printf("Altitude estimada: %.2f m\n", bmp_data.altitude);

        // Leitura do AHT20
        if (aht20_read(I2C0_PORT, &aht_data))
        {
            //printf("Temperatura AHT: %.2f C\n", aht_data.temperature);
            //printf("Umidade: %.2f %%\n\n\n", aht_data.humidity);
        }
        else
        {
            //printf("Erro na leitura do AHT10!\n\n\n");
        }

        xQueueOverwrite(xAHTReadQueue, &aht_data);
            //printf("FILA AHT CHEIA!");
        xQueueOverwrite(xBMPReadQueue, &bmp_data);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
