// Registros do SX1276
#define REG_FIFO            0x00
#define REG_VERSION         0x42
#define REG_OP_MODE         0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_PAYLOAD_LENGTH  0x22
#define REG_IRQ_FLAGS       0x12
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_MODEM_CONFIG3   0x26
#define REG_20_PREAMBLE_MSB 0x20
#define REG_21_PREAMBLE_LSB 0x21
#define REG_RX_NB_BYTES     0x13

// Define o modo LORA para o chip (o outro é  o FSK/OOK)
#define MODE_LORA           0x80

// Abaixo se encontram os modos de operação do modo LORA
#define MODE_SLEEP          0x00
#define MODE_STDBY          0x01
#define MODE_TX             0x03
#define MODE_RX_CONTINUOS   0x05
