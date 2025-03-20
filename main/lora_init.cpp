#include "lora_init.hpp"
#include "esp_log.h"
#include "globalvars.hpp"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_system.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define TAG "LoRa"

#define PIN_MISO  12
#define PIN_MOSI  11
#define PIN_SCK   13
#define PIN_CS    10
#define PIN_RST   9
#define PIN_DIO0  6

spi_device_handle_t spi;

extern "C" {  // Wrap ESP-IDF C functions for compatibility with C++
    #include "driver/gpio.h"
}

uint8_t spi_transfer(uint8_t address, uint8_t value, bool read) {
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));  // Initialize struct properly for C++

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.length = 8;
    trans.rxlength = read ? 8 : 0;
    trans.tx_data[0] = read ? (address & 0x7F) : (address | 0x80);

    esp_err_t ret = spi_device_transmit(spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Transfer Failed");
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
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void spi_init() {
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = PIN_MISO;
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = PIN_CS;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

void lora_config() {
    lora_reset();

    // Enter sleep mode
    lora_write_reg(0x01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set frequency
    lora_write_reg(0x06, 0xE4);
    lora_write_reg(0x07, 0xC0);
    lora_write_reg(0x08, 0x00);

    // Configure settings
    lora_write_reg(0x0D, 0x00);  // FIFO TX base addr
    lora_write_reg(0x0E, 0x00);  // FIFO RX base addr
    lora_write_reg(0x09, 0xFF);  // Max payload length
    lora_write_reg(0x1D, 0x72);  // BW = 125 kHz, CR = 4/5
    lora_write_reg(0x1E, 0x74);  // SF = 7, CRC on
    lora_write_reg(0x26, 0x04);  // Low Data Rate Optimization

    // Set in standby mode
    lora_write_reg(0x01, 0x81);
    ESP_LOGI(TAG, "LoRa Configured");
}

void lora_send(const char *data) {
    lora_write_reg(0x01, 0x81);  //Set to standby mode

    lora_write_reg(0x0D, 0x00);  //FIFO TX base address
    lora_write_reg(0x0E, 0x00);  //FFO RX base address

    int len = std::strlen(data);
    for (int i = 0; i < len; i++) {
        lora_write_reg(0x00, data[i]);
    }

    lora_write_reg(0x22, len);   // Payload length
    lora_write_reg(0x01, 0x83);  // Set to Tx mode

    // Wait for TxDone
    while (!(lora_read_reg(0x12) & 0x08));

    lora_write_reg(0x12, 0x08);  // Clear TxDone flag

    ESP_LOGI(TAG, "LoRa Packet Sent: %s", data);
}

void lora_send_imu_data() {
    char payload[128];

    std::snprintf(payload, sizeof(payload),
             "Time: %llds, Euler(x:%.2f y:%.2f z:%.2f), Vel(x:%.2f y:%.2f z:%.2f), "
             "Grav(x:%.2f y:%.2f z:%.2f), AngAcc(x:%.2f y:%.2f z:%.2f), LinAcc(x:%.2f y:%.2f z:%.2f), Heap: %u",
             latest_timestamp / 1000000,
             latest_euler_data.x, latest_euler_data.y, latest_euler_data.z,
             latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z,
             latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z,
             latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z,
             latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z,
             esp_get_free_heap_size());

    lora_send(payload);
    ESP_LOGI("LoRa", "Sent IMU Data: %s", payload);
}
