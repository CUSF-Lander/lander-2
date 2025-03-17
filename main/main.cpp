#include <stdio.h>
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
        ESP_LOGI(TAG, "Seconds since boot (s): %lld, euler datapoints %li, Latest Pos: (x: %.2f y: %.2f z: %.2f), Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %u", \
            (latest_timestamp/1000000), euler_counter, latest_position.x, latest_position.y, latest_position.z, \
            latest_euler_data.x, latest_euler_data.y, latest_euler_data.z, \
            latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z, \
            latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z, \
            latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z, \
            latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z, free_heap_size);

        //todo: error not handled when vector size is 0
        //ESP_LOGI(TAG, "Last Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler_data.back().x, euler_data.back().y, euler_data.back().z);
        //ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel_data.back().x, lin_accel_data.back().y, lin_accel_data.back().z);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}

// New state estimation task: 200Hz (5ms period)
void state_estimation(void *pvParameters)
{
    const double dt = 0.01; // seconds 0.005s = (200Hz)
    const double dt_ms = dt*1000; // ms
    static double vx = 0, vy = 0, vz = 0; // local velocity integration
    const double deg2rad = 3.14159265358979323846 / 180.0;
    while (1)
    {
        // read measured linear acceleration and Euler angles (in degrees) from globals
        // Subtract the gravity vector from the linear acceleration
        double ax = latest_lin_accel_data.x - latest_gravity_data.x;
        double ay = latest_lin_accel_data.y - latest_gravity_data.y;
        double az = latest_lin_accel_data.z - latest_gravity_data.z;
        double roll_deg = latest_euler_data.x;
        double pitch_deg = latest_euler_data.y;
        double yaw_deg = latest_euler_data.z;

        // convert Euler angles from degrees to radians
        double roll_rad = roll_deg * deg2rad;
        double pitch_rad = pitch_deg * deg2rad;
        double yaw_rad = yaw_deg * deg2rad;

        // Compute rotation matrix elements for body-to-earth transformation
        double cr = cos(roll_rad), sr = sin(roll_rad);
        double cp = cos(pitch_rad), sp = sin(pitch_rad);
        double cy = cos(yaw_rad), sy = sin(yaw_rad);

        // Rotate acceleration vector (a_earth = R * a_body)
        double a_ex = cy * cp * ax + (cy * sp * sr - sy * cr) * ay + (cy * sp * cr + sy * sr) * az;
        double a_ey = sy * cp * ax + (sy * sp * sr + cy * cr) * ay + (sy * sp * cr - cy * sr) * az;
        double a_ez = -sp * ax + cp * sr * ay + cp * cr * az;

        // Integrate acceleration to update velocity
        vx += a_ex * dt;
        vy += a_ey * dt;
        vz += a_ez * dt;

        // Integrate velocity to update position (global variable)
        latest_position.x += vx * dt;
        latest_position.y += vy * dt;
        latest_position.z += vz * dt;

        // wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}

extern "C" void app_main(void)
{

    // Initialize the IMU with the function imu_init() in imu_init.hpp / .cpp 
    //vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 500ms - see if this solves the strange issue of needing to tempoarily unplug the IMU and then quickly plug it back in before the output reaches row 321 - no it does not
    imu_init();

    // Create the vector logging task
    BaseType_t measure_datarate_task = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }
    
    // Launch state estimation task
    BaseType_t state_estimation_task = xTaskCreatePinnedToCore(state_estimation, "state estimation", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if(state_estimation_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create state estimation task!");
    } else {
        ESP_LOGI(TAG, "State estimation task started.");
    }

    while (1)
    {
        // delay time is irrelevant, we just don't want to trip WDT
        vTaskDelay(100UL / portTICK_PERIOD_MS); //originally 10000UL
    }
}