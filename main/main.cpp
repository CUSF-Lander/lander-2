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

char pmem[512] = {0}; // Buffer to store the CPU usage data

// RTOS Task to log the current stats of the data from the IMU every 500ms
void measure_datarate(void *pvParameters)
{   
    while (1) {
        size_t free_heap_size = esp_get_free_heap_size(); // check the amount of ram left

        // Check CPU Usage (commented as per original)
        /*uint32_t cpu_usage_core_0 = 0;
        uint32_t cpu_usage_core_1 = 0;
        esp_cpu_get_load(ESP_CPU_MAIN, &cpu_usage_core_0); // ESP_CPU_MAIN is Core 0
        esp_cpu_get_load(ESP_CPU_APP, &cpu_usage_core_1);  // ESP_CPU_APP is Core 1*/

        // Ensure latest_timestamp is initialized
        int64_t latest_timestamp = 0;
        /*if (timestamps.size() != 0) {
            latest_timestamp = timestamps.back();
        }*/

        // Logging message with latest data
        ESP_LOGI(TAG, "Seconds since boot (s): %lld, euler datapoints %li, Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %u", 
                 (latest_timestamp/1000000), euler_counter, 
                 latest_euler_data.x, latest_euler_data.y, latest_euler_data.z, 
                 latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z, 
                 latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z, 
                 latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z, 
                 latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z, 
                 free_heap_size);

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

// Receiving LoRa packets task
static void lora_receive_task(void* pvParameters) {
    char buffer[128];
    while (1) {
        int len = lora_receive(buffer, sizeof(buffer));
        if (len > 0) {
            ESP_LOGI(TAG, "Received: %s", buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void)
{
    //imu_init();
    // Initialize LoRa
    spi_init();
    lora_config_t config = {};
    config.frequency = 915000000; // 915 MHz
    config.spreading_factor = 7;
    config.coding_rate = 5;       // 4/5
    config.bandwidth = 7;         // 125 kHz
    lora_config(&config);

    // Create the vector logging task
    BaseType_t measure_datarate_task = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }

    // Create LoRa receive task
    BaseType_t receive_task = xTaskCreatePinnedToCore(lora_receive_task, "lora_receive", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (receive_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa receive task!");
    } else {
        ESP_LOGI(TAG, "LoRa receive task started.");
    }

    while (1)
    { 
        lora_send_imu_data();
        vTaskDelay(1000UL / portTICK_PERIOD_MS);
    }
}