#include "espnow_init.hpp"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "espnow_data.hpp"
#include "globalvars.hpp"
#include <cstring>

static const char *TAG = "ESP_NOW_SENDER";

//MAC address of the receiver
static uint8_t receiver_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Broadcast address

//Flow control variables
static volatile bool send_pending = false;
static uint32_t send_success_count = 0;
static uint32_t send_fail_count = 0;

//Callback function for send status (updated for ESP-IDF v5.5.1)
void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    send_pending = false;
    if (status == ESP_NOW_SEND_SUCCESS) {
        send_success_count++;
    } else {
        send_fail_count++;
    }
    

}

//Initialise Wi-Fi and ESP-NOW for sending
void init_espnow_sender() {
    //Initialise NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //Initialise Wi-Fi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    //Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    //Add peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = 0;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW sender initialized successfully.");
}

void esp_now_send_data() {
    //Skip if previous send is still pending
    if (send_pending) {
        return;
    }
    
    static uint32_t sequence = 0;
    esp_now_data_t data;

    //Initialize the entire struct to zero first
    memset(&data, 0, sizeof(data));
    
    data.sequence = ++sequence;
    
    data.sensor_data[0] = latest_euler_data.x;     // Roll
    data.sensor_data[1] = latest_euler_data.y;     // Pitch
    data.sensor_data[2] = latest_euler_data.z;     // Yaw
    data.sensor_data[3] = latest_lin_accel_data.x;
    data.sensor_data[4] = latest_lin_accel_data.y;
    data.sensor_data[5] = latest_lin_accel_data.z;
    data.sensor_data[6] = (float)latest_position.x;
    data.sensor_data[7] = (float)latest_position.y;
    data.sensor_data[8] = (float)latest_position.z;
    data.sensor_data[9] = (float)pressure;
    
    data.timestamp = (uint32_t)(esp_timer_get_time() / 1000);

    send_pending = true;
    esp_err_t result = esp_now_send(receiver_mac, (uint8_t *)&data, sizeof(data));
    
    if (result != ESP_OK) {
        send_pending = false;
        if (result == ESP_ERR_ESPNOW_NO_MEM) {
            send_fail_count++; // Count buffer full as failure
        } else {
            ESP_LOGE(TAG, "Send error #%lu: %s", sequence, esp_err_to_name(result));
        }
    }
    
    // Log statistics every 100 packets
    if (sequence % 100 == 0) {
        uint32_t total = send_success_count + send_fail_count;
        if (total > 0) {
            ESP_LOGI(TAG, "TX #%lu | Success: %lu/%lu (%.1f%%)", 
                     sequence, send_success_count, total,
                     (send_success_count * 100.0f) / total);
        }
    }
}
