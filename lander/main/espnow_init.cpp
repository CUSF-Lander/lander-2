#include "espnow_init.hpp"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "globalvars.hpp"
#include "comms/comm.h"
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
void on_data_sent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    send_pending = false;
    if (status == ESP_NOW_SEND_SUCCESS) {
        send_success_count++;
    } else {
        send_fail_count++;
    }
}

//callback function for receiving data
void esp_now_cmd_handler(const uint8_t *data, size_t len, const uint8_t *src_mac) {
    if (len == sizeof(esp_now_cmd_t)) {
        esp_now_cmd_t *cmd = (esp_now_cmd_t *)data;
        if (cmd->command == 1) { // ESTOP
            ESP_LOGW(TAG, "ESTOP COMMAND RECEIVED!");
            estop_triggered = true;
        } else if (cmd->command == 2) { //ZERO_IMU
            ESP_LOGW(TAG, "ZERO IMU COMMAND RECEIVED!");
            portENTER_CRITICAL(&global_spinlock);
            latest_velocity = {0.0, 0.0, 0.0};
            latest_position = {0.0, 0.0, 0.0};
            portEXIT_CRITICAL(&global_spinlock);
        }
    }
}

static int64_t last_send_time = 0;
static constexpr int64_t SEND_TIMEOUT_US = 100000; // 100ms

//Initialise Wi-Fi and ESP-NOW for sending
void init_espnow_sender() {
    //Initialise NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(WIFI::wifi_init());
    ESP_ERROR_CHECK(WIFI::esp_now_init());

    WIFI::set_command_callback(esp_now_cmd_handler);
    WIFI::set_send_callback(on_data_sent);
    WIFI::add_peer(receiver_mac);
}

void esp_now_send_data() {
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
    memset(&data, 0, sizeof(data)); data.type = 0x04;
    
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
