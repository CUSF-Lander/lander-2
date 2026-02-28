#include <stdio.h>
#include <inttypes.h>
//#include <esp_system.h> // Include for esp_cpu_get_load
#include <esp_heap_caps.h>
#include "globalvars.hpp"
#include "imu_init.hpp"
#include "i2c/servo.hpp"
#include "i2c/i2c_setup.hpp"
#include "motor_init.hpp"
#include "i2c/bmp390.hpp"
#include "espnow_init.hpp"

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

        // snapshot globals
        portENTER_CRITICAL(&global_spinlock);
        uint64_t t_now = latest_timestamp;
        int32_t ec = euler_counter;
        auto pos = latest_position;
        auto euler = latest_euler_data;
        auto ang_vel = latest_ang_velocity_data;
        auto vel = latest_velocity;
        auto grav = latest_gravity_data;
        auto ang_accel = latest_ang_accel_data;
        auto lin_accel = latest_lin_accel_data;
        portEXIT_CRITICAL(&global_spinlock);

        //logging message with latest data
        if (!estop_triggered) {
            ESP_LOGI(TAG, "Seconds since boot (s): %llu, euler datapoints %" PRId32 ", Latest Pos: (x: %.2f y: %.2f z: %.2f), Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Angular Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Linear Velocity: (x: %.2f y: %.2f z: %.2f) [m/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %" PRIu32, \
                (t_now/1000000), ec, pos.x, pos.y, pos.z, \
                euler.x, euler.y, euler.z, \
                ang_vel.x, ang_vel.y, ang_vel.z, \
                vel.x, vel.y, vel.z, \
                grav.x, grav.y, grav.z, \
                ang_accel.x, ang_accel.y, ang_accel.z, \
                lin_accel.x, lin_accel.y, lin_accel.z, free_heap_size);
        }

        //todo: error not handled when vector size is 0
        //ESP_LOGI(TAG, "Last Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler_data.back().x, euler_data.back().y, euler_data.back().z);
        //ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel_data.back().x, lin_accel_data.back().y, lin_accel_data.back().z);
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 500ms
    }
}

// New state estimation task: 100Hz (10ms period)
void state_estimation(void *pvParameters)
{
    const double dt = 0.01; // seconds 0.01s = (100Hz)
    const uint32_t dt_ms = 10; // ms
    //static double vx = 0, vy = 0, vz = 0; // local velocity integration
    const double deg2rad = 3.14159265358979323846 / 180.0;
    while (1)
    {
        // read measured linear acceleration and Euler angles (in degrees) from globals
        portENTER_CRITICAL(&global_spinlock);
        auto current_lin_accel = latest_lin_accel_data;
        auto current_euler = latest_euler_data;
        auto current_vel = latest_velocity;
        auto current_pos = latest_position;
        portEXIT_CRITICAL(&global_spinlock);

        // Acceleration corrected since original calculation accounted for gravity, even though sensor did that already
        double ax = current_lin_accel.x;
        double ay = current_lin_accel.y;
        double az = current_lin_accel.z;

        double roll_deg = current_euler.x;
        double pitch_deg = current_euler.y;
        double yaw_deg = current_euler.z;

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
        current_vel.x += a_ex * dt;
        current_vel.y += a_ey * dt;
        current_vel.z += a_ez * dt;

        // Integrate velocity to update position (global variable)
        current_pos.x += current_vel.x * dt;
        current_pos.y += current_vel.y * dt;
        current_pos.z += current_vel.z * dt;

        // Update globals
        portENTER_CRITICAL(&global_spinlock);
        latest_velocity = current_vel;
        latest_position = current_pos;
        portEXIT_CRITICAL(&global_spinlock);

        // wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(dt_ms));

    }
}

void esp_now_task(void *pvParameters)
{
    while(1) {
        if (!estop_triggered) {
            esp_now_send_data();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); //send every 10ms
    }
}

void bmp390_task(void *pvParameters)
{
    while (1)
    {
        double t, p, a;
        esp_err_t result = bmp390_get_data(&t, &p, &a);

        if (result == ESP_OK) {
            portENTER_CRITICAL(&global_spinlock);
            temperature = t;
            pressure = p;
            altitude = a;
            portEXIT_CRITICAL(&global_spinlock);

            if (!estop_triggered) {
                ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f Pa, Altitude: %.2f m", t, p, a);
            }
        } else {
            if (!estop_triggered) {
                ESP_LOGE(TAG, "Failed to read data from BMP390 sensor");
            }
        } 
        vTaskDelay(pdMS_TO_TICKS(500)); //Run every 0.5 seconds?
    }
}

void testServo() {
    while(1){
        
        pca9685_set_servo_angle(1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 45);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 135);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 180);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // 110-120 is the start point, 540-550 is the end point
    }
}

extern "C" void app_main(void)
{
    //Initialise I2C with error checking
    esp_err_t i2c_result = i2c_master_init();
    if (i2c_result == ESP_OK) {
        ESP_LOGI(TAG, "I2C initialized successfully");
    } else {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(i2c_result));
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); //Let I2C stabilise
    
    pca9685_init();
    bmp390_init(); 
    init_espnow_sender();

    ESP_LOGI(TAG, "Waiting for IMU power-up...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_init();

    // Create the vector logging task with higher stack
    BaseType_t measure_datarate_result = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 8192, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to start
    
    // Launch state estimation task
    BaseType_t state_estimation_task = xTaskCreatePinnedToCore(state_estimation, "state estimation", 4096, NULL, 2, NULL, APP_CPU_NUM);
    if(state_estimation_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create state estimation task!");
    } else {
        ESP_LOGI(TAG, "State estimation task started.");
    }

    // while (1)
    // {
    //     // delay time is irrelevant, we just don't want to trip WDT
    //     vTaskDelay(100UL / portTICK_PERIOD_MS); //originally 10000UL
    // }

    // Initialize the motor
    /*void all_motor_init(void){
        // Initialize the 2 motors with the specified GPIO pins and RMT channels
        initializeMotor(GPIO_NUM_4, RMT_CHANNEL_0);
        initializeMotor(GPIO_NUM_5, RMT_CHANNEL_1);
    }*/

    //code for RTOS Task
    //TODO IMU: Tempoarily disabled to test the IMU Only
    
    BaseType_t motor_task = xTaskCreatePinnedToCore(init_2_motors, "initialize 2 motors", 4096, NULL, 3, NULL, APP_CPU_NUM);
    if(motor_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor task!");
    } else {
        ESP_LOGI(TAG, "motor task started.");
    }
    

    //reverting to function based code for testing 
    //init_2_motors();

    //move esp_now_task to CPU 0 to avoid watchdog issues on CPU 1
    BaseType_t esp_now_task_handle = xTaskCreatePinnedToCore(esp_now_task, "esp_now_task", 4096, NULL, 4, NULL, PRO_CPU_NUM);
    if(esp_now_task_handle != pdPASS) {
        ESP_LOGE(TAG, "Failed to create esp_now task!");
    } else {
        ESP_LOGI(TAG, "esp_now task started on CPU 0.");
    } 

    //launch BMP390 sensor task
    BaseType_t bmp390_task_handle = xTaskCreatePinnedToCore(bmp390_task, "bmp390_task", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if(bmp390_task_handle != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BMP390 task!");
    } else {
        ESP_LOGI(TAG, "BMP390 task started.");
    }

    //todo:code will never reach here as we are testing the motor in an infinite loop

    while (1)
    {
        // delay time is irrelevant, we just don't want to trip WDT
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}