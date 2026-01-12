#ifndef RF24_STM32_H
#define RF24_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // Chứa định nghĩa HAL và GPIO
#include <stdbool.h>
#include <string.h>

/* --- nRF24L01 Register Map & Commands (Từ nRF24L01.h gốc) --- */
#define NRF_CONFIG      0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define NRF_STATUS      0x07
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

/* Bit Mnemonics */
#define MASK_RX_DR      6
#define MASK_TX_DS      5
#define MASK_MAX_RT     4
#define EN_CRC          3
#define CRCO            2
#define PWR_UP          1
#define PRIM_RX         0
#define ENAA_P5         5
#define ENAA_P4         4
#define ENAA_P3         3
#define ENAA_P2         2
#define ENAA_P1         1
#define ENAA_P0         0
#define RF_DR_LOW       5
#define RF_DR_HIGH      3
#define RF_PWR_LOW      1
#define RF_PWR_HIGH     2

/* Các định nghĩa bit bị thiếu (Đã bổ sung) */
#define DPL_P5          5
#define DPL_P4          4
#define DPL_P3          3
#define DPL_P2          2
#define DPL_P1          1
#define DPL_P0          0
#define EN_DPL          2
#define EN_ACK_PAY      1
#define EN_DYN_ACK      0

/* Instructions */
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define REGISTER_MASK   0x1F
#define ACTIVATE        0x50
#define R_RX_PL_WID     0x60
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define W_ACK_PAYLOAD   0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define RF24_NOP        0xFF

/* --- Enums (Từ RF24.h gốc) --- */

typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;

/* --- Struct Handle --- */

typedef struct {
    // Hardware Handle
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *CE_Port;
    uint16_t          CE_Pin;
    GPIO_TypeDef      *CSN_Port;
    uint16_t          CSN_Pin;

    // Internal State
    bool              p_variant;
    uint8_t           payload_size;
    bool              dynamic_payloads_enabled;
    bool              ack_payloads_enabled;
    uint8_t           addr_width;
    uint8_t           pipe0_reading_address[5];
    uint8_t           config_reg;
} RF24_HandleTypeDef;

/* --- Function Prototypes --- */

// Basic Management
void RF24_Init(RF24_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool RF24_begin(RF24_HandleTypeDef *dev);
bool RF24_isChipConnected(RF24_HandleTypeDef *dev);
void RF24_startListening(RF24_HandleTypeDef *dev);
void RF24_stopListening(RF24_HandleTypeDef *dev);
void RF24_powerDown(RF24_HandleTypeDef *dev);
void RF24_powerUp(RF24_HandleTypeDef *dev);

// Writing / Transmitting
bool RF24_write(RF24_HandleTypeDef *dev, const void *buf, uint8_t len);
bool RF24_writeMulticast(RF24_HandleTypeDef *dev, const void *buf, uint8_t len, const bool multicast);
bool RF24_writeFast(RF24_HandleTypeDef *dev, const void *buf, uint8_t len);
bool RF24_writeBlocking(RF24_HandleTypeDef *dev, const void *buf, uint8_t len, uint32_t timeout);
bool RF24_txStandBy(RF24_HandleTypeDef *dev, uint32_t timeout);
void RF24_writeAckPayload(RF24_HandleTypeDef *dev, uint8_t pipe, const void *buf, uint8_t len);

// Reading / Receiving
bool RF24_available(RF24_HandleTypeDef *dev);
bool RF24_availablePipe(RF24_HandleTypeDef *dev, uint8_t *pipe_num);
void RF24_read(RF24_HandleTypeDef *dev, void *buf, uint8_t len);
uint8_t RF24_getDynamicPayloadSize(RF24_HandleTypeDef *dev);
uint8_t RF24_getPayloadSize(RF24_HandleTypeDef *dev);

// Configuration
void RF24_openWritingPipe(RF24_HandleTypeDef *dev, const uint8_t *address);
void RF24_openReadingPipe(RF24_HandleTypeDef *dev, uint8_t number, const uint8_t *address);
void RF24_closeReadingPipe(RF24_HandleTypeDef *dev, uint8_t pipe);
void RF24_setAddressWidth(RF24_HandleTypeDef *dev, uint8_t a_width);
void RF24_setRetries(RF24_HandleTypeDef *dev, uint8_t delay, uint8_t count);
void RF24_setChannel(RF24_HandleTypeDef *dev, uint8_t channel);
uint8_t RF24_getChannel(RF24_HandleTypeDef *dev);
void RF24_setPayloadSize(RF24_HandleTypeDef *dev, uint8_t size);
bool RF24_setDataRate(RF24_HandleTypeDef *dev, rf24_datarate_e speed);
rf24_datarate_e RF24_getDataRate(RF24_HandleTypeDef *dev);
void RF24_setPALevel(RF24_HandleTypeDef *dev, uint8_t level); // 0=MIN, 3=MAX
uint8_t RF24_getPALevel(RF24_HandleTypeDef *dev);
void RF24_setCRCLength(RF24_HandleTypeDef *dev, rf24_crclength_e length);
rf24_crclength_e RF24_getCRCLength(RF24_HandleTypeDef *dev);
void RF24_disableCRC(RF24_HandleTypeDef *dev);
void RF24_setAutoAck(RF24_HandleTypeDef *dev, bool enable);
void RF24_setAutoAckPipe(RF24_HandleTypeDef *dev, uint8_t pipe, bool enable);
void RF24_enableDynamicPayloads(RF24_HandleTypeDef *dev);
void RF24_disableDynamicPayloads(RF24_HandleTypeDef *dev);
void RF24_enableAckPayload(RF24_HandleTypeDef *dev);
void RF24_enableDynamicAck(RF24_HandleTypeDef *dev);

// Advanced / Low Level
void RF24_maskIRQ(RF24_HandleTypeDef *dev, bool tx_ok, bool tx_fail, bool rx_ready);
void RF24_whatHappened(RF24_HandleTypeDef *dev, bool *tx_ok, bool *tx_fail, bool *rx_ready);
uint8_t RF24_flush_rx(RF24_HandleTypeDef *dev);
uint8_t RF24_flush_tx(RF24_HandleTypeDef *dev);
uint8_t RF24_getStatus(RF24_HandleTypeDef *dev);
bool RF24_testCarrier(RF24_HandleTypeDef *dev);
bool RF24_testRPD(RF24_HandleTypeDef *dev);

#ifdef __cplusplus
}
#endif

#endif // RF24_STM32_H
