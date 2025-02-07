#include <stdio.h>
#include "BNO08x.hpp"
//#include <esp_system.h> // Include for esp_cpu_get_load
#include <esp_heap_caps.h>

static const constexpr char* TAG = "Main";

float counter;

// Data storage (using vectors for dynamic storage)
std::vector<bno08x_euler_angle_t> euler_data;
std::vector<bno08x_gyro_t> velocity_data;
std::vector<bno08x_accel_t> gravity_data;
std::vector<bno08x_accel_t> ang_accel_data;
std::vector<bno08x_accel_t> lin_accel_data;
std::vector<int64_t> timestamps; // Store timestamps for each data point

// RTOS Task to log the size of the vector every 500ms
void measure_datarate(void *pvParameters)
{   
    while (1) {
        counter += 0.1;
        int64_t latest_timestamp;
        size_t free_heap_size = esp_get_free_heap_size(); // check the amount of ram left

        //check CPU Usage
        /*uint32_t cpu_usage_core_0 = 0;
        uint32_t cpu_usage_core_1 = 0;
        esp_cpu_get_load(ESP_CPU_MAIN, &cpu_usage_core_0); // ESP_CPU_MAIN is Core 0
        esp_cpu_get_load(ESP_CPU_APP, &cpu_usage_core_1);  // ESP_CPU_APP is Core 1*/

        if (timestamps.size() != 0) {
            latest_timestamp = timestamps.back();
        } else {
            latest_timestamp = 0;
        }
        
        ESP_LOGI(TAG, "Time (last update) %lld, Euler Vector size: %d, LinAcc Vector Size: %d, Free Heap Size %u", (latest_timestamp/1000000), euler_data.size(), lin_accel_data.size(), free_heap_size);
        ESP_LOGI(TAG, "Last Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler_data.back().x, euler_data.back().y, euler_data.back().z);
        ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel_data.back().x, lin_accel_data.back().y, lin_accel_data.back().z);
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 500ms
    }
}

extern "C" void app_main(void)
{
    static BNO08x imu;

    // initialize imu
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }

    // enable game rotation vector and calibrated gyro reports
    imu.rpt.rv_game.enable(2500UL); // tested up to 1000UL with no issues - 100,000us == 100ms report interval //originally 100000UL
    //imu.rpt.cal_gyro.enable(5000UL);
    //imu.rpt.gravity.enable(5000UL);
    //imu.rpt.accelerometer.enable(5000UL);
    imu.rpt.linear_accelerometer.enable(2500UL);
    // see BNO08x::bno08x_reports_t for all possible reports to enable

    // register callback to execute for all reports, 2 different methods

    // method 1, void input param:
    /*
    imu.register_cb(
            []()
            {
                int64_t current_timestamp = esp_timer_get_time(); // Get timestamp in microseconds

                if (imu.rpt.rv_game.has_new_data())
                {

                    bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
                    //counter+=1;
                    euler_data.push_back(imu.rpt.rv_game.get_euler());
                    timestamps.push_back(current_timestamp); // Store timestamp
                    //ESP_LOGI(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Counter %i", euler.x, euler.y, euler.z, counter);
                    //ESP_LOGI(TAG, "Euler Angle, Counter %i", counter);
                }

                if (imu.rpt.cal_gyro.has_new_data())
                {
                    //bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
                    velocity_data.push_back(imu.rpt.cal_gyro.get());

                    //ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
                }

                if (imu.rpt.gravity.has_new_data())
                {
                    //bno08x_accel_t grav = imu.rpt.gravity.get();
                    gravity_data.push_back(imu.rpt.gravity.get());

                    //ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
                }

                if (imu.rpt.accelerometer.has_new_data())
                {
                    //bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
                    ang_accel_data.push_back(imu.rpt.accelerometer.get());
                    //ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                }

                if (imu.rpt.linear_accelerometer.has_new_data())
                {
                    //bno08x_accel_t lin_accel = imu.rpt.accelerometer.get();
                    lin_accel_data.push_back(imu.rpt.linear_accelerometer.get());
                    //ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                }
            }); */

    // method 2, report ID param (comment method 1 out before commenting this in):
    
    imu.register_cb(
            [](uint8_t rpt_ID)
            {
                static bno08x_euler_angle_t euler;
                static bno08x_gyro_t velocity;
                static bno08x_accel_t grav;
                static bno08x_accel_t ang_accel;
                static bno08x_accel_t lin_accel;
                
                int64_t current_timestamp = esp_timer_get_time(); // Get timestamp in microseconds

                switch (rpt_ID)
                {
                    case SH2_GAME_ROTATION_VECTOR:
                        euler = imu.rpt.rv_game.get_euler();
                        euler_data.push_back(imu.rpt.rv_game.get_euler());
                        timestamps.push_back(current_timestamp); // Store timestamp
                        //ESP_LOGI(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
                        break;

                    case SH2_CAL_GYRO:
                        velocity = imu.rpt.cal_gyro.get();
                        //ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
                        break;

                    case SH2_GRAVITY:
                        grav = imu.rpt.gravity.get();
                        //ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
                        break;

                    case SH2_ACCELEROMETER:
                        ang_accel = imu.rpt.accelerometer.get();
                        //ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                        break;

                    case SH2_LINEAR_ACCELERATION:
                        //lin_accel = imu.rpt.accelerometer.get();
                        lin_accel_data.push_back(imu.rpt.linear_accelerometer.get());
                        //ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                        break;

                    default:

                        break;
                }
            });

    



    // Create the vector logging task
    /*BaseType_t measure_datarate_task = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 2048, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }*/

    while (1)
    {
        // delay time is irrelevant, we just don't want to trip WDT
        vTaskDelay(100UL / portTICK_PERIOD_MS); //originally 10000UL
    }
}