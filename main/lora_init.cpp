#include "lora_init.hpp"
#include "globalvars.hpp"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include <cstdio>
#include <cstring>
#include <cinttypes>
#ifndef ESP_LOG_LEVEL
#define ESP_LOG_LEVEL ESP_LOG_INFO
#endif
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "LoRa"

#define PIN_MISO  12
#define PIN_MOSI  11
#define PIN_SCK   13
#define PIN_CS    10
#define PIN_RST   9
#define PIN_DIO0  6

static spi_device_handle_t spi;

// SPI transfer function (read or write)
static uint8_t spi_transfer(uint8_t address, uint8_t value, bool read) {
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.length = 8;
    trans.rxlength = read ? 8 : 0;
    trans.tx_data[0] = read ? (address & 0x7F) : (address | 0x80);
    if (!read) {
        trans.tx_data[1] = value;
        trans.length = 16;
    }

    esp_err_t ret = spi_device_transmit(spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return read ? trans.rx_data[0] : 0;
}

uint8_t lora_read_reg(uint8_t reg) {
    return spi_transfer(reg, 0x00, true);
}

void lora_write_reg(uint8_t reg, uint8_t value) {
    spi_transfer(reg, value, false);
}

void lora_reset() {
    // Zero-initialize and set fields in order
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_RST);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
    }
    
    gpio_set_level(static_cast<gpio_num_t>(PIN_RST), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(PIN_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void spi_init() {
    // Zero-initialize and set fields in order
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = PIN_CS;
    devcfg.queue_size = 1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
    }
}

void lora_config(const lora_config_t* config) {
    lora_reset();

    // Enter sleep mode
    lora_write_reg(0x01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set frequency
    uint32_t frf = ((uint64_t)config->frequency << 19) / 32000000;
    lora_write_reg(0x06, (frf >> 16) & 0xFF);
    lora_write_reg(0x07, (frf >> 8) & 0xFF);
    lora_write_reg(0x08, frf & 0xFF);

    // Configure settings
    lora_write_reg(0x0D, 0x00); // FIFO TX base addr
    lora_write_reg(0x0E, 0x00); // FIFO RX base addr
    lora_write_reg(0x09, 0xFF); // Max payload length

    // Modem config: BW, CR, SF
    uint8_t bw_cr = ((config->bandwidth & 0x0F) << 4) | (config->coding_rate - 4);
    lora_write_reg(0x1D, bw_cr);
    lora_write_reg(0x1E, (config->spreading_factor << 4) | 0x04); // SF, CRC on
    lora_write_reg(0x26, config->spreading_factor >= 11 ? 0x04 : 0x00); // Low Data Rate Optimization

    // Set to standby mode
    lora_write_reg(0x01, 0x81);
    ESP_LOGI(TAG, "LoRa configured: Freq=%" PRIu32 " Hz, SF=%d, BW=%d kHz, CR=4/%d",
             config->frequency, config->spreading_factor,
             config->bandwidth == 7 ? 125 : (config->bandwidth == 8 ? 250 : 500), config->coding_rate);
}

void lora_send(const char* data) {
    lora_write_reg(0x01, 0x81); // Set to standby mode
    lora_write_reg(0x0D, 0x00); // FIFO TX base addr
    lora_write_reg(0x0E, 0x00); // FIFO RX base addr

    size_t len = std::strlen(data);
    if (len > 255) {
        ESP_LOGE(TAG, "Payload too long: %zu bytes", len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(0x00, data[i]);
    }

    lora_write_reg(0x22, static_cast<uint8_t>(len)); // Payload length
    lora_write_reg(0x01, 0x83); // Set to TX mode

    // Wait for TxDone
    uint32_t timeout = 1000;
    while (!(lora_read_reg(0x12) & 0x08) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (timeout == 0) {
        ESP_LOGE(TAG, "TX timeout");
    }

    lora_write_reg(0x12, 0x08); // Clear TxDone flag
    ESP_LOGI(TAG, "LoRa packet sent: %s", data);
}

int lora_receive(char* buffer, size_t max_len) {
    lora_write_reg(0x01, 0x85); // Set to continuous RX mode

    if (lora_read_reg(0x12) & 0x40) { // Check for RxDone
        uint8_t len = lora_read_reg(0x13); // Read payload length
        if (len > max_len - 1) {
            ESP_LOGE(TAG, "Received payload too long: %d bytes", len);
            lora_write_reg(0x12, 0x40); // Clear RxDone
            return 0;
        }

        uint8_t fifo_addr = lora_read_reg(0x10); // Set FIFO pointer
        lora_write_reg(0x0D, fifo_addr);

        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = static_cast<char>(lora_read_reg(0x00));
        }
        buffer[len] = '\0';

        lora_write_reg(0x12, 0x40); // Clear RxDone

        int8_t snr = lora_read_reg(0x19);
        int16_t rssi = -157 + lora_read_reg(0x1B);
        ESP_LOGI(TAG, "LoRa packet received: %s (RSSI=%d dBm, SNR=%d dB)", buffer, rssi, snr);
        return len;
    }
    return 0;
}

void lora_send_imu_data() {
    char payload[512];
    std::snprintf(payload, sizeof(payload),
                  "Time:%llds,Euler(x:%.2f,y:%.2f,z:%.2f),Vel(x:%.2f,y:%.2f,z:%.2f),"
                  "Grav(x:%.2f,y:%.2f,z:%.2f),AngAcc(x:%.2f,y:%.2f,z:%.2f),"
                  "LinAcc(x:%.2f,y:%.2f,z:%.2f),Heap:%lu",
                  latest_timestamp / 1000000,
                  latest_euler_data.x, latest_euler_data.y, latest_euler_data.z,
                  latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z,
                  latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z,
                  latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z,
                  latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z,
                  esp_get_free_heap_size());

    lora_send(payload);
}