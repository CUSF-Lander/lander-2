#include "motor_init.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Include the DShot library
#include "DShotRMT.h"

static const char *TAG = "motor_init";

// Constants
#define MIN_THROTTLE 48      // Minimum throttle value
#define MAX_THROTTLE 2047    // Maximum throttle value

void initializeMotor(gpio_num_t dshot_gpio, rmt_channel_t rmt_channel)
{
    ESP_LOGI(TAG, "Initializing DShot RMT for motor");
    
    // Create a DShot ESC instance
    DShotRMT esc;
    
    // Install the DShot driver on the specified GPIO pin and RMT channel
    esp_err_t result = esc.install(dshot_gpio, rmt_channel);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver: %d", result);
        return;
    }
    
    // Initialize the ESC
    result = esc.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESC: %d", result);
        return;
    }
    
    ESP_LOGI(TAG, "ESC initialized successfully");
    
    // Calculate 5% throttle value
    // The valid throttle range is from MIN_THROTTLE (48) to MAX_THROTTLE (2047)
    // 5% of the usable range: MIN_THROTTLE + 0.05 * (MAX_THROTTLE - MIN_THROTTLE)
    uint16_t five_percent_throttle = MIN_THROTTLE + (uint16_t)(0.25 * (MAX_THROTTLE - MIN_THROTTLE));
    
    ESP_LOGI(TAG, "Setting throttle to 5%% (value: %d)", five_percent_throttle);
    
    //vTaskDelay(10000);  // 1 tick delay, typically 1ms with default FreeRTOS config
    //vTaskDelay(5000 / portTICK_PERIOD_MS); //5 seconds?
    // Main control loop - send the 5% throttle command continuously
    ESP_LOGI(TAG, "Entering control loop with 5%% throttle");
    for (;;) {
        // Send the 5% throttle command
        esp_err_t throttle_result = esc.sendThrottle(five_percent_throttle);
        if (throttle_result != ESP_OK) {
            ESP_LOGE(TAG, "Error sending throttle command: %d", throttle_result);
        }
        
        // Small delay to prevent overwhelming the ESC with commands
        vTaskDelay(5/portTICK_PERIOD_MS);  // 1 tick delay, typically 1ms with default FreeRTOS config
    }
    
    // Note: This function will never return due to the infinite loop above
}