#include "motor_init.hpp"
#include "esp_log.h"
#include "esp_timer.h"
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

    // Physical motor mapping for the current counter-rotating propeller stack:
    //   ESC 1 / RMT channel 0 / GPIO 4  = bottom propeller
    //   ESC 2 / RMT channel 1 / GPIO 25 = top propeller
    //
    // In the old sequential initialization, the bottom propeller initialized
    // first and the top propeller initialized about five seconds later. The
    // arming loop below now sends zero throttle to both ESCs simultaneously.
    gpio_num_t dshot_gpio = GPIO_NUM_4;
    gpio_num_t dshot_gpio2 = GPIO_NUM_25;
    rmt_channel_t rmt_channel = RMT_CHANNEL_0;
    rmt_channel_t rmt_channel2 = RMT_CHANNEL_1;

    ESP_LOGI(TAG, "Initializing DShot RMT for both motors");
    
    // Create a DShot ESC instance
    DShotRMT esc;
    DShotRMT esc2;
    
    // Install the DShot driver on the specified GPIO pin and RMT channel
    esp_err_t result = esc.install(dshot_gpio, rmt_channel);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ESC 1 DShot driver: %s",
                 esp_err_to_name(result));
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }


    esp_err_t result2 = esc2.install(dshot_gpio2, rmt_channel2);
    if (result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ESC 2 DShot driver: %s",
                 esp_err_to_name(result2));
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

    // Arm both ESCs together. Sequential calls to esc.init() made the second
    // motor start its five-second arming period after the first one finished.
    ESP_LOGI(TAG, "Holding both ESCs at zero throttle for %d ms",
             MOTOR_WIRING_TEST_ZERO_HOLD_MS);
    const TickType_t arm_start = xTaskGetTickCount();
    const TickType_t arm_duration =
        pdMS_TO_TICKS(MOTOR_WIRING_TEST_ZERO_HOLD_MS);
    const TickType_t packet_interval = pdMS_TO_TICKS(10);

    while ((xTaskGetTickCount() - arm_start) < arm_duration) {
        result = esc.sendThrottle(0);
        result2 = esc2.sendThrottle(0);
        if (result != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG,
                     "Failed to send arming packets (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(result), esp_err_to_name(result2));
            ESP_LOGE(TAG, "Motor output disabled");
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(packet_interval);
    }

    ESP_LOGI(TAG, "Both ESCs armed successfully");

    // Calculate 5% throttle value
    // The valid throttle range is from MIN_THROTTLE (48) to MAX_THROTTLE (2047)
    // 5% of the usable range: MIN_THROTTLE + 0.05 * (MAX_THROTTLE - MIN_THROTTLE)
    uint16_t throttle_percent = MIN_THROTTLE + (uint16_t)((MOTOR_WIRING_TEST_THROTTLE_PERCENT / 100.0f) * (MAX_THROTTLE - MIN_THROTTLE));
    
    ESP_LOGI(TAG, "Setting throttle to %d%% (value: %d)", MOTOR_WIRING_TEST_THROTTLE_PERCENT, throttle_percent);
    

    // Main control loop - send the throttle command continuously
#if MOTOR_WIRING_TEST_MODE
    ESP_LOGW(TAG, "MOTOR_WIRING_TEST_MODE enabled: ignoring ground-station timeout and software ESTOP");
#endif
    ESP_LOGI(TAG, "Entering control loop with %d%% throttle", MOTOR_WIRING_TEST_THROTTLE_PERCENT);
    while (true) {
#if !MOTOR_WIRING_TEST_MODE
        // Automatically trigger ESTOP if no messages from GS in 500ms
        if (last_gs_msg_time > 0 && (esp_timer_get_time() - last_gs_msg_time) > 500000) {
            if (!estop_triggered) {
                ESP_LOGE(TAG, "Ground station connection lost (500ms timeout)! Triggering ESTOP.");
                estop_triggered = true;
            }
        }

        if (estop_triggered) {
            //send 0 throttle to stop motors
            esc.sendThrottle(0);
            esc2.sendThrottle(0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
#endif

        // Send the throttle command to both ESCs and fail safe to zero if
        // either transmission reports an error.
        result = esc.sendThrottle(throttle_percent);
        result2 = esc2.sendThrottle(throttle_percent);
        if (result != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG,
                     "DShot transmission failed (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(result), esp_err_to_name(result2));
            throttle_percent = 0;
        }
        
        vTaskDelay(packet_interval); // 100 Hz update rate
    }
    
    // Note: This function will never return due to the infinite loop above
}
