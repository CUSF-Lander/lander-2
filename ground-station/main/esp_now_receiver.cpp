#include "esp_now_receiver.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "espnow_data.hpp"

static const char *TAG = "ESP-NOW-RX";

#define LANDER_DEVICE_PIN (gpio_num_t)27

//HW ESTOP Button Pin (BOOT button is usually GPIO 0)
#define HW_ESTOP_PIN (gpio_num_t)0

#define LANDER_TIMEOUT_MS 3000
static esp_timer_handle_t lander_timeout_timer = NULL;
static bool lander_device_active = false;

static uint8_t lander_mac[6] = {0};
static bool lander_mac_known = false;

static void init_lander_device_pin(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LANDER_DEVICE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)LANDER_DEVICE_PIN, 0); //start with device OFF
    ESP_LOGI(TAG, "Lander device pin %d initialized", (int)LANDER_DEVICE_PIN);
}

static void control_lander_device(bool enable)
{
    gpio_set_level((gpio_num_t)LANDER_DEVICE_PIN, enable ? 1 : 0);
    lander_device_active = enable;
    ESP_LOGI(TAG, "Lander device %s (Pin %d)", enable ? "ENABLED" : "DISABLED", (int)LANDER_DEVICE_PIN);
}

static void lander_timeout_callback(void* arg)
{
    if (lander_device_active) {
        ESP_LOGW(TAG, "*** LANDER TIMEOUT - NO MESSAGES RECEIVED - TURNING OFF ***");
        control_lander_device(false);
    }
}

static void init_lander_timeout_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &lander_timeout_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lander_timeout",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &lander_timeout_timer);
    ESP_LOGI(TAG, "Lander timeout timer initialized (%d ms)", LANDER_TIMEOUT_MS);
}

static uint32_t packets_received = 0;
static int32_t packets_lost = 0;
static uint32_t last_sequence = 0;
static uint32_t first_sequence = 0;

static uint32_t total_bytes_received = 0;
static uint32_t bandwidth_start_time = 0;

#define WINDOW_SECS 10
struct {
    uint32_t received;
    uint32_t lost;
} stat_window[WINDOW_SECS];
static uint32_t current_sec = 0;
static bool stat_window_init = false;

void esp_now_advanced_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    static bool gpio_initialized = false;
    static bool first_packet = true;

    if (!gpio_initialized) {
        init_lander_device_pin();
        init_lander_timeout_timer();
        gpio_initialized = true;
    }

    if (!stat_window_init) {
        for (int i=0; i<WINDOW_SECS; i++) {
            stat_window[i].received = 0;
            stat_window[i].lost = 0;
        }
        stat_window_init = true;
    }

    if (!lander_mac_known) {
        memcpy(lander_mac, recv_info->src_addr, 6);
        lander_mac_known = true;

        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, lander_mac, 6);
        peer_info.channel = 1;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        if (esp_now_add_peer(&peer_info) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer");
        } else {
            ESP_LOGI(TAG, "Added lander peer for commands");
        }
    }

    if (!lander_device_active) {
        control_lander_device(true);
    }
    esp_timer_stop(lander_timeout_timer);
    esp_timer_start_once(lander_timeout_timer, LANDER_TIMEOUT_MS * 1000);

    packets_received++;

    if (len == sizeof(esp_now_data_t)) {
        esp_now_data_t *received_data = (esp_now_data_t *)data;
        
        uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000);
        if (now_sec != current_sec) {
            uint32_t diff = now_sec - current_sec;
            if (diff >= WINDOW_SECS) diff = WINDOW_SECS;
            for (uint32_t i = 1; i <= diff; i++) {
                uint32_t idx = (current_sec + i) % WINDOW_SECS;
                stat_window[idx].received = 0;
                stat_window[idx].lost = 0;
            }
            current_sec = now_sec;
        }

        uint32_t this_packet_lost = 0;

        if (first_packet) {
            first_sequence = received_data->sequence;
            first_packet = false;
        } else if (received_data->sequence < last_sequence || received_data->sequence > last_sequence + 1000) {
            // Sequence reset (sender reset or heavy loss) - treat as new stream
            first_sequence = received_data->sequence;
            ESP_LOGW(TAG, "*** SEQUENCE RESET: %lu -> %lu ***", last_sequence, received_data->sequence);
        } else if (received_data->sequence != last_sequence + 1) {
            int32_t lost = received_data->sequence - last_sequence - 1;
            if (lost > 0) {
                packets_lost += lost;
                this_packet_lost = (uint32_t)lost;
                ESP_LOGW(TAG, "*** PACKET LOSS: %ld packets (seq %lu -> %lu) ***", (long)lost, last_sequence, received_data->sequence);
            }
        }
        last_sequence = received_data->sequence;
        
        stat_window[current_sec % WINDOW_SECS].received += 1;
        stat_window[current_sec % WINDOW_SECS].lost += this_packet_lost;

        uint32_t window_received = 0;
        uint32_t window_lost = 0;
        for (int i = 0; i < WINDOW_SECS; i++) {
            window_received += stat_window[i].received;
            window_lost += stat_window[i].lost;
        }

        float success_rate = 100.0f;
        uint32_t window_total = window_received + window_lost;
        if (window_total > 0) {
            success_rate = (window_received * 100.0f) / window_total;
        }

        if (packets_received % 50 == 0) {
            printf("{\"type\":\"telemetry\",\"seq\":%lu,\"rssi\":%d,\"success_rate\":%.1f,\"lost\":%ld,"
                   "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
                   "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
                   "\"px\":%.2f,\"py\":%.2f,\"pz\":%.2f,\"pressure\":%.0f}\n",
                   received_data->sequence, recv_info->rx_ctrl->rssi, success_rate, packets_lost,
                   received_data->roll, received_data->pitch, received_data->yaw,
                   received_data->lin_acc_x, received_data->lin_acc_y, received_data->lin_acc_z,
                   received_data->pos_x, received_data->pos_y, received_data->pos_z,
                   received_data->pressure);
            fflush(stdout);
        }
               
    } else {
        ESP_LOGW(TAG, "Unexpected data length: %d (expected %d)", len, sizeof(esp_now_data_t));
    }

    if (bandwidth_start_time == 0) {
        bandwidth_start_time = esp_timer_get_time();
    }
    total_bytes_received += len;

    uint32_t elapsed_time_ms = (esp_timer_get_time() - bandwidth_start_time) / 1000;
    if (elapsed_time_ms >= 5000) {
        float bandwidth_kbps = (total_bytes_received * 8.0f) / elapsed_time_ms;
        printf("{\"type\":\"bandwidth\",\"kbps\":%.2f,\"total_bytes\":%lu,\"elapsed_sec\":%.1f}\n",
               bandwidth_kbps, total_bytes_received, elapsed_time_ms / 1000.0f);
        fflush(stdout);
        total_bytes_received = 0;
        bandwidth_start_time = esp_timer_get_time();
    }
}

void esp_now_send_estop(void)
{
    if (!lander_mac_known) {
        ESP_LOGE(TAG, "Cannot send ESTOP: Lander MAC unknown");
        return;
    }
    
    esp_now_cmd_t cmd;
    cmd.command = 1;
    
    esp_err_t result = esp_now_send(lander_mac, (uint8_t *)&cmd, sizeof(cmd));
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "ESTOP command sent successfully");
        printf("{\"type\":\"estop_status\",\"status\":\"sent\"}\n");
        fflush(stdout);
    } else {
        ESP_LOGE(TAG, "Error sending ESTOP command: %s", esp_err_to_name(result));
        printf("{\"type\":\"estop_status\",\"status\":\"error\"}\n");
        fflush(stdout);
    }
}

void esp_now_send_zero_imu(void)
{
    if (!lander_mac_known) {
        ESP_LOGE(TAG, "Cannot send ZERO IMU: Lander MAC unknown");
        return;
    }
    
    esp_now_cmd_t cmd;
    cmd.command = 2;
    
    esp_err_t result = esp_now_send(lander_mac, (uint8_t *)&cmd, sizeof(cmd));
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "ZERO IMU command sent successfully");
        printf("{\"type\":\"zero_imu_status\",\"status\":\"sent\"}\n");
        fflush(stdout);
    } else {
        ESP_LOGE(TAG, "Error sending ZERO IMU command: %s", esp_err_to_name(result));
        printf("{\"type\":\"zero_imu_status\",\"status\":\"error\"}\n");
        fflush(stdout);
    }
}

#include <sys/select.h>
#include <fcntl.h>

static void hw_estop_task(void *pvParameter)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HW_ESTOP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (1) {
        if (gpio_get_level(HW_ESTOP_PIN) == 0) {
            ESP_LOGW(TAG, "HW ESTOP TRIGGERED!");
            esp_now_send_estop();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void sw_estop_task(void *pvParameter)
{
    char line[128];
    int pos = 0;

    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[pos] = '\0';
                if (strcmp(line, "ESTOP") == 0) {
                    ESP_LOGW(TAG, "SW ESTOP TRIGGERED!");
                    esp_now_send_estop();
                } else if (strcmp(line, "ZERO_IMU") == 0) {
                    ESP_LOGW(TAG, "SW ZERO IMU TRIGGERED!");
                    esp_now_send_zero_imu();
                }
                pos = 0;
            } else if (pos < sizeof(line) - 1) {
                line[pos++] = c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void init_estop_task(void)
{
    if (!uart_is_driver_installed(UART_NUM_0)) {
        uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
        uart_vfs_dev_use_driver(UART_NUM_0);
    }

    xTaskCreate(hw_estop_task, "hw_estop_task", 2048, NULL, 5, NULL);
    xTaskCreate(sw_estop_task, "sw_estop_task", 4096, NULL, 5, NULL);
}

esp_err_t esp_now_receiver_stats_reset(void)
{
    packets_received = 0;
    packets_lost = 0;
    last_sequence = 0;
    first_sequence = 0;
    total_bytes_received = 0;
    bandwidth_start_time = 0;
    ESP_LOGI(TAG, "Statistics reset");
    return ESP_OK;
}

void esp_now_receiver_print_stats(void)
{
    ESP_LOGI(TAG, "=== ESP-NOW Receiver Statistics ===");
    ESP_LOGI(TAG, "Packets Received: %lu", packets_received);
    ESP_LOGI(TAG, "Packets Lost: %lu", packets_lost);
    if (packets_received + packets_lost > 0) {
        ESP_LOGI(TAG, "Success Rate: %.1f%%", 
                 (float)(packets_received * 100) / (packets_received + packets_lost));
    }
    ESP_LOGI(TAG, "Last Sequence: %lu", last_sequence);
    ESP_LOGI(TAG, "Total Bytes Received: %lu", total_bytes_received);
    ESP_LOGI(TAG, "===================================");
}