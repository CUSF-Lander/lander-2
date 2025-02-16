#include <stdio.h>
#include "BNO08x.hpp"
//#include <esp_system.h> // Include for esp_cpu_get_load
#include <esp_heap_caps.h>
#include "globalvars.hpp"
#include "imu_init.hpp"

static const constexpr char* TAG = "Main";


char pmem[512] = {0}; // Buffer to store the CPU usage data


// RTOS Task to log the current stats of the data from the IMU every 500ms
void measure_datarate(void *pvParameters)
{   
    
    while (1) {
        size_t free_heap_size = esp_get_free_heap_size(); // check the amount of ram left

        //check CPU Usage
        /*uint32_t cpu_usage_core_0 = 0;
        uint32_t cpu_usage_core_1 = 0;
        esp_cpu_get_load(ESP_CPU_MAIN, &cpu_usage_core_0); // ESP_CPU_MAIN is Core 0
        esp_cpu_get_load(ESP_CPU_APP, &cpu_usage_core_1);  // ESP_CPU_APP is Core 1*/

        /*if (timestamps.size() != 0) {
            latest_timestamp = timestamps.back();
        } else {
            latest_timestamp = 0;
        }*/

        //ESP_LOGI(TAG, "ABOUT TO PRINT");
        /*vTaskGetRunTimeStats(pmem);
        ESP_LOGI(TAG, "CPU Usage: %s", pmem);*/
        
        //original logging message with vectors
        //ESP_LOGI(TAG, "Seconds since boot (s): %lld, Vector Sizes: Euler: %d, AngVel: %d, Grav: %d, AngAccel: %d, LinAcc: %d, Free Heap Size %u", (latest_timestamp/1000000), euler_data.size(), velocity_data.size(), gravity_data.size(), ang_accel_data.size(),lin_accel_data.size(), free_heap_size);

        //logging message with latest data
        ESP_LOGI(TAG, "Seconds since boot (s): %lld, euler datapoints %li, Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %u", (latest_timestamp/1000000), euler_counter, latest_euler_data.x, latest_euler_data.y, latest_euler_data.z, latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z, latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z, latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z, latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z, free_heap_size);

        //todo: error not handled when vector size is 0
        //ESP_LOGI(TAG, "Last Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler_data.back().x, euler_data.back().y, euler_data.back().z);
        //ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel_data.back().x, lin_accel_data.back().y, lin_accel_data.back().z);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

extern "C" void app_main(void)
{

    // Initialize the IMU with the function imu_init() in imu_init.hpp / .cpp   
    imu_init();


    //vTaskDelay(1000UL / portTICK_PERIOD_MS); //to ensure the first data is collected before the vector logging task starts - not a robust solution
    // Create the vector logging task
    BaseType_t measure_datarate_task = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }

    while (1)
    {
        // delay time is irrelevant, we just don't want to trip WDT
        vTaskDelay(100UL / portTICK_PERIOD_MS); //originally 10000UL
    }
}