#include "motor_init.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "globalvars.hpp"

// Include the DShot library
#include "DShotRMT.h"

static const char *TAG = "motor_init";

// Constants
#define MIN_THROTTLE 48      // Minimum throttle value
#define MAX_THROTTLE 2047    // Maximum throttle value

void init_2_motors(void* pvParameters)
//void init_2_motors()
{
    /*
    initializeMotor(GPIO_NUM_4, RMT_CHANNEL_0);
        initializeMotor(GPIO_NUM_5, RMT_CHANNEL_1);
        */

    //pin configurations: (changed motor2 to 18 as IMU was also initialised there)
    gpio_num_t dshot_gpio = GPIO_NUM_4;
    gpio_num_t dshot_gpio2 = GPIO_NUM_18;
    rmt_channel_t rmt_channel = RMT_CHANNEL_0;
    rmt_channel_t rmt_channel2 = RMT_CHANNEL_1;

    ESP_LOGI(TAG, "Initializing DShot RMT for both motors");
    
    // Create a DShot ESC instance
    DShotRMT esc;
    DShotRMT esc2;
    
    // Install the DShot driver on the specified GPIO pin and RMT channel
    esp_err_t result = esc.install(dshot_gpio, rmt_channel);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver: %d", result);
        //return;
    }


    esp_err_t result2 = esc2.install(dshot_gpio2, rmt_channel2);
    if (result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver: %d", result2);
        //return;
    }
    
    // Initialize the ESC
    result = esc.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESC 1: %d", result);
        //return;
    }

    
    result2 = esc2.init();
    if (result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESC 2: %d", result2);
        //return;
    }
    

    ESP_LOGI(TAG, "Both ESCs initialized successfully");
    
    // Calculate 5% throttle value
    // The valid throttle range is from MIN_THROTTLE (48) to MAX_THROTTLE (2047)
    // 5% of the usable range: MIN_THROTTLE + 0.05 * (MAX_THROTTLE - MIN_THROTTLE)
    uint16_t throttle_percent = MIN_THROTTLE + (uint16_t)(0.05 * (MAX_THROTTLE - MIN_THROTTLE));
    
    ESP_LOGI(TAG, "Setting throttle to 5%% (value: %d)", throttle_percent);
    

    // Main control loop - send the 5% throttle command continuously
    ESP_LOGI(TAG, "Entering control loop with 5%% throttle");
    while (true) {
        if (estop_triggered) {
            //send 0 throttle to stop motors
            esc.sendThrottle(0);
            esc2.sendThrottle(0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Send the throttle % command
        esp_err_t throttle_result = esc.sendThrottle(throttle_percent);
        /*if (throttle_result != ESP_OK) {
            ESP_LOGE(TAG, "Error sending throttle command: %d", throttle_result);
        }*/
        //ESP_LOGI(TAG, "Throttle command sent to first ESC: %d", throttle_result);

        // Send the throttle % command to the second ESC
        esp_err_t throttle_result2 = esc2.sendThrottle(throttle_percent); 
        /*if (throttle_result2 != ESP_OK) {
            ESP_LOGE(TAG, "Error sending throttle command: %d", throttle_result);
        }*/
        //ESP_LOGI(TAG, "Throttle command sent to second ESC: %d", throttle_result2);
        
        // Small delay to prevent overwhelming the ESC with commands
        vTaskDelay(pdMS_TO_TICKS(10));  // 1 tick delay, typically 1ms with default FreeRTOS config
    }
    
    // Note: This function will never return due to the infinite loop above
}
