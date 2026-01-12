/*
 * RF24_STM32.c
 *
 *  Created on: Jan 1, 2026
 *      Author: khanh
 */

#include "RF24_STM32.h"

// Macro tiện ích
#define _BV(x) (1 << (x))
#define rf24_min(a, b) ((a) < (b) ? (a) : (b))
#define rf24_max(a, b) ((a) > (b) ? (a) : (b))

// Hàm Delay micro giây (Ước lượng, nếu cần chính xác hãy dùng DWT hoặc Timer)
static void delayMicroseconds(uint32_t us) {
    // Giả sử chạy ở 72MHz hoặc cao hơn, vòng lặp này chỉ mang tính ước lượng.
    // Với STM32 HAL, HAL_Delay tính bằng ms.
    // Dưới 1ms, ta dùng vòng lặp blocking.
    uint32_t count = us * (SystemCoreClock / 1000000U / 4); // Chia 4 là ước lượng số cycle/loop
    while(count--);
}

// --- Low Level SPI ---

static void csn(RF24_HandleTypeDef *dev, bool mode) {
    if (mode) {
        HAL_GPIO_WritePin(dev->CSN_Port, dev->CSN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dev->CSN_Port, dev->CSN_Pin, GPIO_PIN_RESET);
    }
}

static void ce(RF24_HandleTypeDef *dev, bool level) {
    if (level) {
        HAL_GPIO_WritePin(dev->CE_Port, dev->CE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dev->CE_Port, dev->CE_Pin, GPIO_PIN_RESET);
    }
}

static uint8_t read_register(RF24_HandleTypeDef *dev, uint8_t reg) {
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = (R_REGISTER | (REGISTER_MASK & reg));
    tx_data[1] = RF24_NOP;

    csn(dev, 0);
    HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, 2, 100);
    csn(dev, 1);

    return rx_data[1];
}

static void read_register_bytes(RF24_HandleTypeDef *dev, uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t cmd = (R_REGISTER | (REGISTER_MASK & reg));

    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, buf, len, 100);
    csn(dev, 1);
}

static void write_register(RF24_HandleTypeDef *dev, uint8_t reg, uint8_t value) {
    uint8_t tx_data[2];

    tx_data[0] = (W_REGISTER | (REGISTER_MASK & reg));
    tx_data[1] = value;

    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, tx_data, 2, 100);
    csn(dev, 1);
}

static void write_register_bytes(RF24_HandleTypeDef *dev, uint8_t reg, const uint8_t* buf, uint8_t len) {
    uint8_t cmd = (W_REGISTER | (REGISTER_MASK & reg));

    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(dev->hspi, (uint8_t*)buf, len, 100);
    csn(dev, 1);
}

static void write_payload(RF24_HandleTypeDef *dev, const void* buf, uint8_t len, const uint8_t writeType) {
    uint8_t cmd = writeType;
    uint8_t data_len = len;
    uint8_t blank_len = 0;

    if (!dev->dynamic_payloads_enabled) {
        data_len = rf24_min(len, dev->payload_size);
        blank_len = dev->payload_size - data_len;
    } else {
        data_len = rf24_min(len, 32);
    }

    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(dev->hspi, (uint8_t*)buf, data_len, 100);
    // Zero padding if static payload
    if (blank_len > 0) {
        uint8_t zero = 0;
        for(uint8_t i = 0; i < blank_len; i++) {
            HAL_SPI_Transmit(dev->hspi, &zero, 1, 100);
        }
    }
    csn(dev, 1);
}

static void read_payload(RF24_HandleTypeDef *dev, void* buf, uint8_t len) {
    uint8_t cmd = R_RX_PAYLOAD;
    uint8_t data_len = len;
    uint8_t blank_len = 0;

    if (!dev->dynamic_payloads_enabled) {
        data_len = rf24_min(len, dev->payload_size);
        blank_len = dev->payload_size - data_len;
    } else {
        data_len = rf24_min(len, 32);
    }

    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, (uint8_t*)buf, data_len, 100);
    // Read and discard padding
    if (blank_len > 0) {
        uint8_t dummy;
        for(uint8_t i = 0; i < blank_len; i++) {
             HAL_SPI_Receive(dev->hspi, &dummy, 1, 100);
        }
    }
    csn(dev, 1);
}

static void toggle_features(RF24_HandleTypeDef *dev) {
    uint8_t cmd = ACTIVATE;
    uint8_t data = 0x73;
    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(dev->hspi, &data, 1, 100);
    csn(dev, 1);
}

// --- Public Initialization ---

void RF24_Init(RF24_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    dev->hspi = hspi;
    dev->CE_Port = ce_port;
    dev->CE_Pin = ce_pin;
    dev->CSN_Port = csn_port;
    dev->CSN_Pin = csn_pin;

    dev->p_variant = false;
    dev->payload_size = 32;
    dev->dynamic_payloads_enabled = false;
    dev->ack_payloads_enabled = false;
    dev->addr_width = 5;
    memset(dev->pipe0_reading_address, 0, 5);
}

bool RF24_begin(RF24_HandleTypeDef *dev) {
    // Reset pins
    ce(dev, 0);
    csn(dev, 1);
    HAL_Delay(5); // Wait for radio to settle

    // Setup retries
    RF24_setRetries(dev, 5, 15);
    RF24_setDataRate(dev, RF24_1MBPS);

    // Toggle features to check for p_variant
    uint8_t before_toggle = read_register(dev, FEATURE);
    toggle_features(dev);
    uint8_t after_toggle = read_register(dev, FEATURE);
    dev->p_variant = (before_toggle == after_toggle);

    if (after_toggle) {
        if (dev->p_variant) {
            // module did not power on reset
            toggle_features(dev);
        }
        write_register(dev, FEATURE, 0);
    }

    dev->ack_payloads_enabled = false;
    write_register(dev, DYNPD, 0);
    dev->dynamic_payloads_enabled = false;
    write_register(dev, EN_AA, 0x3F);
    write_register(dev, EN_RXADDR, 3);
    RF24_setPayloadSize(dev, 32);
    RF24_setAddressWidth(dev, 5);
    RF24_setChannel(dev, 76);

    // Clear status
    write_register(dev, NRF_STATUS, _BV(MASK_RX_DR) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT));

    RF24_flush_rx(dev);
    RF24_flush_tx(dev);

    write_register(dev, NRF_CONFIG, _BV(EN_CRC) | _BV(CRCO));
    dev->config_reg = read_register(dev, NRF_CONFIG);

    RF24_powerUp(dev);

    // Check if connected
    return (dev->config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)));
}

bool RF24_isChipConnected(RF24_HandleTypeDef *dev) {
    uint8_t setup_aw = read_register(dev, SETUP_AW);
    return (setup_aw == (dev->addr_width - 2));
}

// --- Power Management ---

void RF24_powerDown(RF24_HandleTypeDef *dev) {
    ce(dev, 0);
    dev->config_reg &= ~_BV(PWR_UP);
    write_register(dev, NRF_CONFIG, dev->config_reg);
}

void RF24_powerUp(RF24_HandleTypeDef *dev) {
    if (!(dev->config_reg & _BV(PWR_UP))) {
        dev->config_reg |= _BV(PWR_UP);
        write_register(dev, NRF_CONFIG, dev->config_reg);
        // Wait 1.5ms per datasheet (Crystal start up) but 5ms is safer
        HAL_Delay(5);
    }
}

// --- Operation ---

void RF24_startListening(RF24_HandleTypeDef *dev) {
    RF24_powerUp(dev);
    dev->config_reg |= _BV(PRIM_RX);
    write_register(dev, NRF_CONFIG, dev->config_reg);
    write_register(dev, NRF_STATUS, _BV(MASK_RX_DR) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT));
    ce(dev, 1);

    // Restore pipe 0 if needed
    if (dev->pipe0_reading_address[0] > 0) {
         write_register_bytes(dev, RX_ADDR_P0, dev->pipe0_reading_address, dev->addr_width);
    } else {
        RF24_closeReadingPipe(dev, 0);
    }
}

void RF24_stopListening(RF24_HandleTypeDef *dev) {
    ce(dev, 0);
    delayMicroseconds(100);
    if(dev->ack_payloads_enabled) {
        RF24_flush_tx(dev);
    }

    dev->config_reg &= ~_BV(PRIM_RX);
    write_register(dev, NRF_CONFIG, dev->config_reg);

    // Enable RX on pipe 0 for Auto-Ack
    write_register(dev, EN_RXADDR, read_register(dev, EN_RXADDR) | _BV(0));
}

// --- Writing ---

bool RF24_write(RF24_HandleTypeDef *dev, const void *buf, uint8_t len) {
    return RF24_writeMulticast(dev, buf, len, 0);
}

bool RF24_writeMulticast(RF24_HandleTypeDef *dev, const void *buf, uint8_t len, const bool multicast) {
    // Start writing
    RF24_writeFast(dev, buf, len);

    // Wait until complete
    while (!((RF24_getStatus(dev)) & (_BV(MASK_TX_DS) | _BV(MASK_MAX_RT)))) {
        // Optional: Implement timeout here to prevent hang
    }

    ce(dev, 0);

    uint8_t status = RF24_getStatus(dev);
    write_register(dev, NRF_STATUS, _BV(MASK_RX_DR) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT));

    if (status & _BV(MASK_MAX_RT)) {
        RF24_flush_tx(dev);
        return false;
    }
    return true;
}

bool RF24_writeFast(RF24_HandleTypeDef *dev, const void *buf, uint8_t len) {
    write_payload(dev, buf, len, W_TX_PAYLOAD);
    ce(dev, 1);
    return true;
}

bool RF24_writeBlocking(RF24_HandleTypeDef *dev, const void *buf, uint8_t len, uint32_t timeout) {
    uint32_t start = HAL_GetTick();

    while ((RF24_getStatus(dev) & _BV(MASK_TX_DS)) == 0 && (RF24_getStatus(dev) & _BV(MASK_MAX_RT)) == 0) {
        if (HAL_GetTick() - start > timeout) {
            return false;
        }
    }

    // Start writing
    RF24_writeFast(dev, buf, len);
    return true;
}

bool RF24_txStandBy(RF24_HandleTypeDef *dev, uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    while (!(read_register(dev, FIFO_STATUS) & _BV(4))) { // TX_EMPTY
        if (HAL_GetTick() - start > timeout) {
             return false;
        }
    }
    ce(dev, 0);
    return true;
}

void RF24_writeAckPayload(RF24_HandleTypeDef *dev, uint8_t pipe, const void *buf, uint8_t len) {
    if (dev->ack_payloads_enabled) {
        write_payload(dev, buf, len, W_ACK_PAYLOAD | (pipe & 0x07));
    }
}

// --- Reading ---

bool RF24_available(RF24_HandleTypeDef *dev) {
    return RF24_availablePipe(dev, NULL);
}

bool RF24_availablePipe(RF24_HandleTypeDef *dev, uint8_t *pipe_num) {
    uint8_t status = RF24_getStatus(dev);
    bool result = (status & _BV(MASK_RX_DR));

    if (result) {
        if (pipe_num) {
            *pipe_num = (status >> 1) & 0x07;
        }
        // Clear IRQ
        write_register(dev, NRF_STATUS, _BV(MASK_RX_DR));

        // Handle ACK payload case? (Simplified from original library)
        if (status & _BV(MASK_TX_DS)) {
             write_register(dev, NRF_STATUS, _BV(MASK_TX_DS));
        }
    }

    return result || !(read_register(dev, FIFO_STATUS) & _BV(0)); // RX_EMPTY
}

void RF24_read(RF24_HandleTypeDef *dev, void *buf, uint8_t len) {
    read_payload(dev, buf, len);
    // Clear IRQ
    write_register(dev, NRF_STATUS, _BV(MASK_RX_DR));
}

uint8_t RF24_getDynamicPayloadSize(RF24_HandleTypeDef *dev) {
    return read_register(dev, R_RX_PL_WID);
}

uint8_t RF24_getPayloadSize(RF24_HandleTypeDef *dev) {
    return dev->payload_size;
}

// --- Pipes & Addressing ---

void RF24_openWritingPipe(RF24_HandleTypeDef *dev, const uint8_t *address) {
    write_register_bytes(dev, RX_ADDR_P0, address, dev->addr_width);
    write_register_bytes(dev, TX_ADDR, address, dev->addr_width);
    write_register(dev, RX_PW_P0, dev->payload_size);
}

void RF24_openReadingPipe(RF24_HandleTypeDef *dev, uint8_t number, const uint8_t *address) {
    if (number == 0) {
        memcpy(dev->pipe0_reading_address, address, dev->addr_width);
    }

    if (number <= 5) {
        if (number < 2) {
             write_register_bytes(dev, RX_ADDR_P0 + number, address, dev->addr_width);
        } else {
             write_register(dev, RX_ADDR_P0 + number, address[0]);
        }

        write_register(dev, RX_PW_P0 + number, dev->payload_size);
        write_register(dev, EN_RXADDR, read_register(dev, EN_RXADDR) | _BV(number));
    }
}

void RF24_closeReadingPipe(RF24_HandleTypeDef *dev, uint8_t pipe) {
    write_register(dev, EN_RXADDR, read_register(dev, EN_RXADDR) & ~_BV(pipe));
}

void RF24_setAddressWidth(RF24_HandleTypeDef *dev, uint8_t a_width) {
    dev->addr_width = a_width;
    write_register(dev, SETUP_AW, (a_width - 2));
}

// --- Configuration ---

void RF24_setRetries(RF24_HandleTypeDef *dev, uint8_t delay, uint8_t count) {
    write_register(dev, SETUP_RETR, (delay & 0xF) << 4 | (count & 0xF));
}

void RF24_setChannel(RF24_HandleTypeDef *dev, uint8_t channel) {
    write_register(dev, RF_CH, channel);
}

uint8_t RF24_getChannel(RF24_HandleTypeDef *dev) {
    return read_register(dev, RF_CH);
}

void RF24_setPayloadSize(RF24_HandleTypeDef *dev, uint8_t size) {
    dev->payload_size = rf24_min(size, 32);
}

bool RF24_setDataRate(RF24_HandleTypeDef *dev, rf24_datarate_e speed) {
    uint8_t setup = read_register(dev, RF_SETUP);
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    if (speed == RF24_250KBPS) {
        setup |= _BV(RF_DR_LOW);
    } else if (speed == RF24_2MBPS) {
        setup |= _BV(RF_DR_HIGH);
    }
    // 1MBPS is 00

    write_register(dev, RF_SETUP, setup);
    return (read_register(dev, RF_SETUP) == setup);
}

rf24_datarate_e RF24_getDataRate(RF24_HandleTypeDef *dev) {
    uint8_t dr = read_register(dev, RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    if (dr == _BV(RF_DR_LOW)) return RF24_250KBPS;
    if (dr == _BV(RF_DR_HIGH)) return RF24_2MBPS;
    return RF24_1MBPS;
}

void RF24_setPALevel(RF24_HandleTypeDef *dev, uint8_t level) {
    uint8_t setup = read_register(dev, RF_SETUP) & 0xF8;
    if (level > 3) level = 3;
    level = (level << 1) + 1; // +1 bit LNA
    write_register(dev, RF_SETUP, setup | level);
}

uint8_t RF24_getPALevel(RF24_HandleTypeDef *dev) {
    return (read_register(dev, RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

void RF24_setCRCLength(RF24_HandleTypeDef *dev, rf24_crclength_e length) {
    uint8_t config = read_register(dev, NRF_CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));

    if (length == RF24_CRC_DISABLED) {
        // Do nothing
    } else if (length == RF24_CRC_8) {
        config |= _BV(EN_CRC);
    } else {
        config |= _BV(EN_CRC) | _BV(CRCO);
    }
    write_register(dev, NRF_CONFIG, config);
    dev->config_reg = config;
}

rf24_crclength_e RF24_getCRCLength(RF24_HandleTypeDef *dev) {
    uint8_t config = read_register(dev, NRF_CONFIG);
    if (config & _BV(EN_CRC)) {
        if (config & _BV(CRCO)) return RF24_CRC_16;
        return RF24_CRC_8;
    }
    return RF24_CRC_DISABLED;
}

void RF24_disableCRC(RF24_HandleTypeDef *dev) {
    uint8_t config = read_register(dev, NRF_CONFIG) & ~_BV(EN_CRC);
    write_register(dev, NRF_CONFIG, config);
    dev->config_reg = config;
}

void RF24_setAutoAck(RF24_HandleTypeDef *dev, bool enable) {
    if (enable)
        write_register(dev, EN_AA, 0x3F);
    else
        write_register(dev, EN_AA, 0);
}

void RF24_setAutoAckPipe(RF24_HandleTypeDef *dev, uint8_t pipe, bool enable) {
    if (pipe <= 5) {
        uint8_t en_aa = read_register(dev, EN_AA);
        if (enable) {
            en_aa |= _BV(pipe);
        } else {
            en_aa &= ~_BV(pipe);
        }
        write_register(dev, EN_AA, en_aa);
    }
}

void RF24_enableDynamicPayloads(RF24_HandleTypeDef *dev) {
    write_register(dev, FEATURE, read_register(dev, FEATURE) | _BV(EN_DPL));
    write_register(dev, DYNPD, read_register(dev, DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));
    dev->dynamic_payloads_enabled = true;
}

void RF24_disableDynamicPayloads(RF24_HandleTypeDef *dev) {
    write_register(dev, FEATURE, 0);
    write_register(dev, DYNPD, 0);
    dev->dynamic_payloads_enabled = false;
    dev->ack_payloads_enabled = false;
}

void RF24_enableAckPayload(RF24_HandleTypeDef *dev) {
    dev->ack_payloads_enabled = true;
    write_register(dev, FEATURE, read_register(dev, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));
    write_register(dev, DYNPD, read_register(dev, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
    dev->dynamic_payloads_enabled = true;
}

void RF24_enableDynamicAck(RF24_HandleTypeDef *dev) {
    write_register(dev, FEATURE, read_register(dev, FEATURE) | _BV(EN_DYN_ACK));
}

// --- Status & Debugging ---

uint8_t RF24_getStatus(RF24_HandleTypeDef *dev) {
    uint8_t status = 0;
    csn(dev, 0);
    status = RF24_NOP;
    HAL_SPI_TransmitReceive(dev->hspi, &status, &status, 1, 100);
    csn(dev, 1);
    return status;
}

void RF24_maskIRQ(RF24_HandleTypeDef *dev, bool tx_ok, bool tx_fail, bool rx_ready) {
    uint8_t config = read_register(dev, NRF_CONFIG);
    config &= ~(_BV(MASK_MAX_RT) | _BV(MASK_TX_DS) | _BV(MASK_RX_DR));
    config |= (!tx_fail << MASK_MAX_RT) | (!tx_ok << MASK_TX_DS) | (!rx_ready << MASK_RX_DR);
    write_register(dev, NRF_CONFIG, config);
    dev->config_reg = config;
}

void RF24_whatHappened(RF24_HandleTypeDef *dev, bool *tx_ok, bool *tx_fail, bool *rx_ready) {
    uint8_t status = RF24_getStatus(dev);
    write_register(dev, NRF_STATUS, _BV(MASK_RX_DR) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT));
    *tx_ok = status & _BV(MASK_TX_DS);
    *tx_fail = status & _BV(MASK_MAX_RT);
    *rx_ready = status & _BV(MASK_RX_DR);
}

uint8_t RF24_flush_rx(RF24_HandleTypeDef *dev) {
    uint8_t cmd = FLUSH_RX;
    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    csn(dev, 1);
    return read_register(dev, NRF_STATUS);
}

uint8_t RF24_flush_tx(RF24_HandleTypeDef *dev) {
    uint8_t cmd = FLUSH_TX;
    csn(dev, 0);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    csn(dev, 1);
    return read_register(dev, NRF_STATUS);
}

bool RF24_testCarrier(RF24_HandleTypeDef *dev) {
    return (read_register(dev, CD) & 1);
}

bool RF24_testRPD(RF24_HandleTypeDef *dev) {
    return (read_register(dev, 0x09) & 1); // RPD
}
