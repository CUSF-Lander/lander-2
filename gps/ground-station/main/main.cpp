#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "comm.h"
#include "uart.h"
static const char* TAG = "GS";




const uint8_t rover_mac[6] = {0xf8, 0xb3, 0xb7, 0x20, 0x13, 0x64}; // Replace with actual rover MAC if known
const uint8_t* mac_ptr = rover_mac;

extern "C" void app_main(void)
{
    // NVS must be up before WiFi starts
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(WIFI::wifi_init());
    ESP_ERROR_CHECK(WIFI::esp_now_init());

    // We don't know the rover's MAC yet, so register the broadcast peer.
    // After the first pong arrives the rover's MAC is auto-learned and all
    // subsequent pings are sent directly to it.

    // Print our own MAC so you can hard-code it on the rover side if needed.
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Ground station MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // 3c:8a:1f:9c:d5:38

    WIFI::add_peer(mac_ptr);

    ESP_LOGI(TAG, "Sending a ping every 2 s — waiting for rover...");
    Uart::init();
    xTaskCreate(Uart::rx_task, "uart_rx_task", 1024 * 16, NULL, configMAX_PRIORITIES - 1, NULL);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
