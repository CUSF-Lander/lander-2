#include <cstdio>
#include "globalvars.hpp"
#include "imu_init.hpp"
#include "lora_init.hpp"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <soc/soc.h>
#ifndef ESP_LOG_LEVEL
#define ESP_LOG_LEVEL ESP_LOG_INFO
#endif
#include "esp_log.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1
#endif

static const constexpr char* TAG = "Main";

char pmem[512] = {0};

// Task to log the current stats of the IMU data every 500ms.
void measure_datarate(void *pvParameters)
{
    int iteration = 0;
    while (1) {
        iteration++;
        ESP_LOGD(TAG, "measure_datarate: Starting iteration %d", iteration);
        size_t free_heap_size = esp_get_free_heap_size(); // Check amount of free RAM
        ESP_LOGD(TAG, "measure_datarate: Free heap size = %u", free_heap_size);

        // Ensure latest_timestamp is initialized
        int64_t latest_timestamp = 0;

        ESP_LOGI(TAG, "Seconds since boot (s): %lld, euler datapoints %li, Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %u", 
                 (latest_timestamp/1000000), euler_counter, 
                 latest_euler_data.x, latest_euler_data.y, latest_euler_data.z, 
                 latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z, 
                 latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z, 
                 latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z, 
                 latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z, 
                 free_heap_size);

        ESP_LOGD(TAG, "measure_datarate: Finished iteration %d", iteration);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay 500ms
    }
}

// Task to receive LoRa packets.
static void lora_receive_task(void* pvParameters) {
    char buffer[128];
    while (1) {
        ESP_LOGD(TAG, "lora_receive_task: Waiting to receive packet");
        int len = lora_receive(buffer, sizeof(buffer));
        if (len > 0) {
            ESP_LOGI(TAG, "Received: %s", buffer);
        } else {
            ESP_LOGD(TAG, "lora_receive_task: No packet received in this cycle");
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Task to send IMU data via LoRa.
static void lora_send_task(void* pvParameters) {
    while (1) { 
        ESP_LOGD(TAG, "lora_send_task: Preparing to send IMU data");
        lora_send_imu_data();
        ESP_LOGD(TAG, "lora_send_task: Finished sending IMU data");
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Started app_main()");
    //imu_init();
    // Initialize LoRa
    spi_init();
    lora_config_t config = {};
    config.frequency = 905000000; // 915 MHz
    config.spreading_factor = 7;
    config.coding_rate = 5;       // 4/5
    config.bandwidth = 7;         // 125 kHz
    lora_config(&config);

    // Create the IMU data logging task.
    /*
    BaseType_t measure_datarate_task = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create measure_datarate task!");
    } else {
        ESP_LOGI(TAG, "measure_datarate task started.");
    }

    BaseType_t receive_task = xTaskCreatePinnedToCore(lora_receive_task, "lora_receive", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (receive_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa receive task!");
    } else {
        ESP_LOGI(TAG, "LoRa receive task started.");
    }
    */
    
    // Create the LoRa send task.
    BaseType_t send_task = xTaskCreatePinnedToCore(lora_send_task, "lora_send", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (send_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa send task!");
    } else {
        ESP_LOGI(TAG, "LoRa send task started.");
    }
}