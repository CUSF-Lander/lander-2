#include "espnow_init.hpp"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "globalvars.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

static const char *TAG = "ESP_NOW_SENDER";

//MAC address of the receiver
static uint8_t receiver_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Broadcast address

//Flow control variables
static std::atomic<bool> send_pending(false);
static std::atomic<uint32_t> send_success_count(0);
static std::atomic<uint32_t> send_fail_count(0);

//callback function for send status
void on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    send_pending = false;
    if (status == ESP_NOW_SEND_SUCCESS) {
        send_success_count++;
    } else {
        send_fail_count++;
    }
}

//callback function for receiving data
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(esp_now_cmd_t)) {
        esp_now_cmd_t *cmd = (esp_now_cmd_t *)data;
        if (cmd->command == 1) { // ESTOP
            ESP_LOGW(TAG, "ESTOP COMMAND RECEIVED!");
            estop_triggered = true;
        }
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    //set Long Range Mode (LR) and Max TX Power (improves range but decreases throughput, need to find balance)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_wifi_start());
    
    //explicitly set channel 1 to ensure sender/receiver match (must be called after esp_wifi_start)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    //set max tx power (must be called after esp_wifi_start)
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(82)); 

    //Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    //Add peer (if one doesn't already exist)
    if (!esp_now_is_peer_exist(receiver_mac)) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, receiver_mac, 6);
        peer_info.channel = 1; //match the wifi channel set above
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }

    ESP_LOGI(TAG, "ESP-NOW sender initialized successfully.");
}

void esp_now_send_data() {
    static int64_t last_send_time = 0;
    const int64_t SEND_TIMEOUT_US = 50000; //50ms watchdog

    //Skip if previous send is still pending, unless timeout
    if (send_pending) {
        if ((esp_timer_get_time() - last_send_time) > SEND_TIMEOUT_US) {
            send_pending = false;
            send_fail_count++;
        } else {
            return;
        }
    }
    
    static uint32_t sequence = 0;
    esp_now_data_t data;

    //Initialize the entire struct to zero first
    memset(&data, 0, sizeof(data));
    
    data.sequence = ++sequence;
    data.timestamp = (uint32_t)(esp_timer_get_time() / 1000); // ms

    //protect global variable access with a spinlock/critical section- 64 bit double reads are not guaranteed atomic on 32-bit ESP32
    portENTER_CRITICAL(&global_spinlock);
    
    data.roll      = latest_euler_data.x;
    data.pitch     = latest_euler_data.y;
    data.yaw       = latest_euler_data.z;
    
    data.lin_acc_x = latest_lin_accel_data.x;
    data.lin_acc_y = latest_lin_accel_data.y;
    data.lin_acc_z = latest_lin_accel_data.z;
    
    data.pos_x     = (float)latest_position.x;
    data.pos_y     = (float)latest_position.y;
    data.pos_z     = (float)latest_position.z;
    
    data.pressure  = (float)pressure;
    
    portEXIT_CRITICAL(&global_spinlock);

    send_pending = true;
    last_send_time = esp_timer_get_time();
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
        uint32_t success = send_success_count.load();
        uint32_t fail = send_fail_count.load();
        uint32_t total = success + fail;
        if (total > 0) {
            ESP_LOGI(TAG, "TX #%lu | Success: %lu/%lu (%.1f%%)", 
                     sequence, success, total,
                     (success * 100.0f) / total);
        }
    }
}
