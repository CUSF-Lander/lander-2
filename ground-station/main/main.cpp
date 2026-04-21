#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_now_receiver.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "espnow_data.hpp"

static const char *TAG = "ESP-NOW-RECEIVER";

void esp_now_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    ESP_LOGI(TAG, "Received data from: %s, len: %d", mac_str, len);

    if (len == sizeof(esp_now_data_t)) {
        esp_now_data_t *received_data = (esp_now_data_t *)data;
        ESP_LOGI(TAG, "Seq: %lu | Timestamp: %lu ms", received_data->sequence, received_data->timestamp);
        ESP_LOGI(TAG, "Euler (r/p/y): %.2f, %.2f, %.2f deg", 
                 received_data->roll, received_data->pitch, received_data->yaw);
        ESP_LOGI(TAG, "Lin Accel (x/y/z): %.2f, %.2f, %.2f m/s²", 
                 received_data->lin_acc_x, received_data->lin_acc_y, received_data->lin_acc_z);
        ESP_LOGI(TAG, "Position (x/y/z): %.2f, %.2f, %.2f | Pressure: %.2f Pa", 
                 received_data->pos_x, received_data->pos_y, 
                 received_data->pos_z, received_data->pressure);
    } else {
        ESP_LOGW(TAG, "Received data with unexpected length: %d", len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    }
}

//initialise WiFi in STA mode for ESP-NOW
esp_err_t wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    //match sender's long range mode (LR)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    
    ESP_ERROR_CHECK(esp_wifi_start());
    
    //explicitly set channel 1 to ensure sender/receiver match
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi initialized for ESP-NOW");
    return ESP_OK;
}

esp_err_t esp_now_init_receiver(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_advanced_recv_callback));
    
    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
    return ESP_OK;
}

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Starting ESP-NOW Ground Station Receiver");

    // Initialize WiFi
    ESP_ERROR_CHECK(wifi_init());

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init_receiver());

    // Initialize ESTOP task
    init_estop_task();

    // Print MAC address for reference
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Receiver MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "ESP-NOW receiver ready. Waiting for data...");

    // Main loop - keep the task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}