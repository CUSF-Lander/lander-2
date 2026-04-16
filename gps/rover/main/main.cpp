#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "comm.h"
#include "uart.h"

static const char* TAG = "ROVER";

const uint8_t bradcast_mac[6] = {0x3c, 0x8a, 0x1f, 0x9c, 0xd5, 0x38};
const uint8_t* mac_ptr = bradcast_mac;
// f8:b3:b7:20:13:64
void on_correction_received(const uint8_t* data, uint8_t len, const uint8_t* src_mac) {
    ESP_LOGI(TAG, "Received correction data from ground station");
    // Here you would typically parse the RTCM bytes and apply corrections to your GPS processing.
    // For this test, we'll just log the raw data length.

    // Example: print the first few bytes of the correction data
    /*
    ESP_LOGI(TAG, "Correction data (first 16 bytes or less):");
    for (int i = 0; i < len && i < 16; ++i) {
        ESP_LOGI(TAG, "%02x ", data[i]);
    }
    */
    ESP_LOGI(TAG, "Correction data length: %d bytes", len);
    ESP_LOGI(TAG, "Source MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    ESP_LOGI(TAG, "Correction data (first 16 bytes or less):");
    for (int i = 0; i < len && i < 16; ++i) {
        ESP_LOGI(TAG, "%02x ", data[i]);
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(WIFI::wifi_init());
    ESP_ERROR_CHECK(WIFI::esp_now_init());

    // Print our own MAC so you can hard-code it on the ground station if needed.
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Rover MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


    ESP_LOGI(TAG, "Ready — waiting for pings from ground station...");
    
    WIFI::add_peer(mac_ptr);
    WIFI::set_correction_callback(on_correction_received);


    // Nothing to do in the main loop for a connectivity test.
    // In the real application this is where you'd also start the UART task.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
