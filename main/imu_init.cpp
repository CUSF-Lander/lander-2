#include "imu_init.hpp"
#include "esp_log.h"
#include "globalvars.hpp"
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

static const constexpr char* TAG = "IMU_INIT";

void imu_init()
{   
    static BNO08x imu;

    ESP_LOGI(TAG, "Starting IMU initialisation...");
    
    //try initialisation with retries for ESP-IDF 5.5.1 compatibility
    const int max_attempts = 3;
    bool init_success = false;
    
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        ESP_LOGI(TAG, "IMU initialisation attempt %d/%d", attempt, max_attempts);
        
        if (imu.initialize()) {
            init_success = true;
            ESP_LOGI(TAG, "IMU initialised successfully!");
            break;
        }
        
        ESP_LOGW(TAG, "Attempt %d failed", attempt);
        if (attempt < max_attempts) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    
    if (!init_success) {
        ESP_LOGE(TAG, "IMU initialisation failed after %d attempts!", max_attempts);
        return;
    }

    // IMU - set data reporting rates
    int report_rate = 10000UL; // 5500us == 5ms report interval
    //imu.rpt.rv_game.enable(report_rate); // tested up to 1000UL with no issues - 100,000us == 100ms report interval //originally 100000UL
    imu.rpt.rv.enable(report_rate);  // Standard Rotation Vector instead of rv_game - this one fuses data with the magnetometer
    imu.rpt.cal_gyro.enable(report_rate);
    imu.rpt.gravity.enable(report_rate);
    imu.rpt.accelerometer.enable(report_rate);
    imu.rpt.linear_accelerometer.enable(report_rate);
    imu.rpt.cal_magnetometer.enable(1000000UL); // 1000ms report interval - magnetometer calibration quality report
    // see BNO08x::bno08x_reports_t for all possible reports to enable

    // register callback to execute for all reports, 2 different methods
    // method 2 from the example - using a case switch - report ID param
    
    imu.register_cb(
            [](uint8_t rpt_ID)
            {
                static bno08x_euler_angle_t euler;
                static bno08x_gyro_t velocity;
                static bno08x_accel_t grav;
                static bno08x_accel_t ang_accel;
                static bno08x_accel_t lin_accel;
                static bno08x_magf_t mag_cal_quality;
                
                int64_t current_timestamp = esp_timer_get_time(); // Get timestamp in microseconds

                switch (rpt_ID)
                {
                    case SH2_ROTATION_VECTOR:
                        euler = imu.rpt.rv.get_euler();
                        
                        portENTER_CRITICAL(&global_spinlock);
                        latest_euler_data = euler;
                        latest_timestamp = current_timestamp;
                        euler_counter +=1;
                        portEXIT_CRITICAL(&global_spinlock);

                        //euler_data.push_back(euler);
                        //timestamps.push_back(current_timestamp); // Store timestamp
                        //ESP_LOGI(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
                        break;

                    case SH2_CAL_GYRO:
                        velocity = imu.rpt.cal_gyro.get();
                        // velocity_data.push_back(velocity);
                        portENTER_CRITICAL(&global_spinlock);
                        latest_ang_velocity_data = velocity;
                        portEXIT_CRITICAL(&global_spinlock);

                        //ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
                        break;

                    case SH2_GRAVITY:
                        grav = imu.rpt.gravity.get();
                        //gravity_data.push_back(grav);
                        portENTER_CRITICAL(&global_spinlock);
                        latest_gravity_data = grav;
                        portEXIT_CRITICAL(&global_spinlock);
                        //ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
                        break;

                    case SH2_ACCELEROMETER:
                        ang_accel = imu.rpt.accelerometer.get();
                        portENTER_CRITICAL(&global_spinlock);
                        latest_ang_accel_data = ang_accel;
                        portEXIT_CRITICAL(&global_spinlock);
                        //ang_accel_data.push_back(ang_accel);
                        //ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                        break;

                    case SH2_LINEAR_ACCELERATION:
                        lin_accel = imu.rpt.linear_accelerometer.get();
                        portENTER_CRITICAL(&global_spinlock);
                        latest_lin_accel_data = lin_accel;
                        portEXIT_CRITICAL(&global_spinlock);
                        //lin_accel_data.push_back(imu.rpt.linear_accelerometer.get());
                        //ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                        break;
                    
                    case SH2_MAGNETIC_FIELD_CALIBRATED:
                        mag_cal_quality = imu.rpt.cal_magnetometer.get();
                        portENTER_CRITICAL(&global_spinlock);
                        latest_mag_cal_quality = mag_cal_quality;
                        portEXIT_CRITICAL(&global_spinlock);
                        // mag_data.accuracy will show calibration quality (0-3)
                        break;
                    

                    default:

                        break;
                }
            });

}


//old way of storing data (no longer used - need to put in effort to set up global vars if needed)
// Data storage (using vectors for dynamic storage)

/*std::vector<bno08x_euler_angle_t> euler_data;
std::vector<bno08x_gyro_t> velocity_data;
std::vector<bno08x_accel_t> gravity_data;
std::vector<bno08x_accel_t> ang_accel_data;
std::vector<bno08x_accel_t> lin_accel_data;
std::vector<int64_t> timestamps; // Store timestamps for each data point
*/