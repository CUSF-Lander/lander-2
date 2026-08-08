#include "motor_init.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "globalvars.hpp"

// Include the DShot library
#include "DShotRMT.h"

static const char *TAG = "motor_init";

struct motor_pin_change_request_t {
    uint8_t motor_idx;
    gpio_num_t new_pin;
};

static QueueHandle_t motor_pin_change_queue = nullptr;

// Constants
#define MIN_THROTTLE 48      // Minimum throttle value
#define MAX_THROTTLE 2047    // Maximum throttle value

// Global ESC instances so they can be re-initialized remotely
static DShotRMT global_esc1;
static DShotRMT global_esc2;

static uint16_t power_percent_to_throttle(uint8_t power_percent)
{
    if (power_percent == 0) {
        return 0;
    }

    if (power_percent > 100) {
        power_percent = 100;
    }

    return MIN_THROTTLE + (uint16_t)((power_percent * (MAX_THROTTLE - MIN_THROTTLE)) / 100);
}

// Reject pins unsafe for DShot before tearing down the old pin: 34-39
// (input-only), 6-11 (flash), 0/12/15 (strapping), 1/3 (UART0), 2 (sensor power).
static bool is_valid_dshot_pin(gpio_num_t pin)
{
    const int p = (int)pin;

    if (p < 0 || p > 33) {
        ESP_LOGE(TAG, "Invalid DShot GPIO %d (valid: 0-33, output-capable)", p);
        return false;
    }
    if ((p >= 6 && p <= 11) || p == 0 || p == 1 || p == 2 || p == 3 || p == 12 || p == 15) {
        ESP_LOGE(TAG, "Refusing DShot GPIO %d: reserved (flash, strapping, UART0 or sensor power)", p);
        return false;
    }

    return true;
}

void reinit_motor_pin(uint8_t motor_idx, gpio_num_t new_pin) {
    ESP_LOGW(TAG, "Reinitializing motor %d to GPIO %d", motor_idx, new_pin);
    DShotRMT *target = nullptr;
    if (motor_idx == 0) {
        target = &global_esc1;
    } else if (motor_idx == 1) {
        target = &global_esc2;
    } else {
        ESP_LOGW(TAG, "Ignoring invalid motor index %d", motor_idx);
        return;
    }

    if (target == nullptr) {
        return;
    }

    if (!is_valid_dshot_pin(new_pin)) {
        ESP_LOGE(TAG, "Ignoring SET_PIN for motor %d: GPIO %d is not usable for DShot",
                 motor_idx, (int)new_pin);
        return;
    }

    // Stop driving the old pin before releasing the RMT channel.
    target->sendThrottle(0);
    vTaskDelay(pdMS_TO_TICKS(20));

    esp_err_t result = target->uninstall();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall DShot driver for motor %d: %d", motor_idx, result);
        return;
    }

    result = target->install(new_pin);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver for motor %d on GPIO %d: %d", motor_idx, new_pin, result);
        return;
    }

    // NOTE: init() holds zero throttle for DSHOT_ARM_DELAY (~5 s), blocking this
    // task. The other ESC receives no DShot frames during that window (BLHeli_S
    // may fail-safe and need re-arming), and the 500 ms GS ESTOP watchdog is
    // skipped meanwhile. Bench-use only.
    result = target->init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize motor %d on GPIO %d: %d", motor_idx, new_pin, result);
    }
}

void request_motor_pin_change(uint8_t motor_idx, gpio_num_t new_pin)
{
    if (motor_pin_change_queue == nullptr) {
        ESP_LOGW(TAG, "Motor pin change requested before queue init");
        return;
    }

    motor_pin_change_request_t request = {
        .motor_idx = motor_idx,
        .new_pin = new_pin,
    };

    if (xQueueSend(motor_pin_change_queue, &request, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Motor pin change queue full, dropping request for motor %d", motor_idx);
    }
}

// Send zero-throttle DShot frames to both ESCs for duration_ms. Used for
// simultaneous ESC arming and (optionally) before/after direction commands.
static esp_err_t hold_both_zero_ms(uint32_t duration_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t duration = pdMS_TO_TICKS(duration_ms);
    const TickType_t interval = pdMS_TO_TICKS(10);

    while ((xTaskGetTickCount() - start) < duration) {
        esp_err_t r1 = global_esc1.sendThrottle(0);
        esp_err_t r2 = global_esc2.sendThrottle(0);
        if (r1 != ESP_OK || r2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send zero throttle (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(r1), esp_err_to_name(r2));
            return ESP_ERR_INVALID_STATE;
        }
        vTaskDelay(interval);
    }
    return ESP_OK;
}

void init_2_motors(void* pvParameters)
{
    // Pin configurations: GPIO 4 for motor 1, GPIO 25 for motor 2
    gpio_num_t dshot_gpio = GPIO_NUM_4;
    gpio_num_t dshot_gpio2 = GPIO_NUM_25;

    ESP_LOGI(TAG, "Initializing DShot RMT for both motors");

    esp_err_t result = global_esc1.install(dshot_gpio);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver for motor 1: %s", esp_err_to_name(result));
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

    esp_err_t result2 = global_esc2.install(dshot_gpio2);
    if (result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver for motor 2: %s", esp_err_to_name(result2));
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

    // Arm both ESCs simultaneously. (Sequential arming made the second motor
    // start its arming period ~5 s after the first one finished.)
    ESP_LOGI(TAG, "Holding both ESCs at zero throttle for %d ms", MOTOR_ARM_ZERO_HOLD_MS);
    if (hold_both_zero_ms(MOTOR_ARM_ZERO_HOLD_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

#if MOTOR_REVERSE_BOTH_ON_STARTUP
    // BLHeli_S command 21 is a temporary reversal relative to the direction
    // saved in ESC flash. Send it at every startup, while both motors remain
    // at zero throttle. Ten identical packets exceed the six-packet minimum.
    ESP_LOGW(TAG, "Applying temporary DShot reversal to both motors");

    if (hold_both_zero_ms(MOTOR_DIRECTION_COMMAND_ZERO_HOLD_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

    for (int i = 0; i < MOTOR_DIRECTION_COMMAND_REPEATS; i++) {
        result = global_esc1.sendDirectionCommand(true);
        result2 = global_esc2.sendDirectionCommand(true);
        if (result != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send direction command (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(result), esp_err_to_name(result2));
            ESP_LOGE(TAG, "Motor output disabled");
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (hold_both_zero_ms(MOTOR_DIRECTION_COMMAND_SETTLE_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "DShot reversal commands sent to both motors; "
             "direction must be verified with propellers removed");
#endif // MOTOR_REVERSE_BOTH_ON_STARTUP

    if (motor_pin_change_queue == nullptr) {
        motor_pin_change_queue = xQueueCreate(4, sizeof(motor_pin_change_request_t));
    }

    static bool logged_first_throttle = false;

    // Main control loop: drive both ESCs at the throttle set by the ground
    // station power slider. Pins stay on GPIO 4/25 unless a SET_PIN command
    // arrives over ESP-NOW (queued via request_motor_pin_change).
    ESP_LOGI(TAG, "Entering control loop");
    while (true) {
        motor_pin_change_request_t request;
        while (motor_pin_change_queue != nullptr && xQueueReceive(motor_pin_change_queue, &request, 0) == pdTRUE) {
            if (!estop_triggered.load()) {
                // Pin changes re-arm an ESC (~5 s) and block this task: only while ESTOP'd.
                ESP_LOGW(TAG, "Rejecting SET_PIN for motor %d while motors are armed", request.motor_idx);
                continue;
            }
            reinit_motor_pin(request.motor_idx, request.new_pin);
        }

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
            global_esc1.sendThrottle(0);
            global_esc2.sendThrottle(0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
#endif // !MOTOR_WIRING_TEST_MODE

#if MOTOR_WIRING_TEST_MODE
        // Bench wiring-test mode: fixed throttle, ignores ground station.
        uint16_t throttle_percent = MIN_THROTTLE +
            (uint16_t)((MOTOR_WIRING_TEST_THROTTLE_PERCENT / 100.0f) * (MAX_THROTTLE - MIN_THROTTLE));
        ESP_LOGW(TAG, "MOTOR_WIRING_TEST_MODE enabled: ignoring ground-station timeout and software ESTOP");
#else
        uint8_t power_percent = motor_power_percent.load();
        uint16_t throttle_percent = power_percent_to_throttle(power_percent);
#endif

        if (!logged_first_throttle) {
            ESP_LOGI(TAG, "About to send first throttle frame: throttle=%u", throttle_percent);
        }

        // Send the throttle command to both ESCs and fail safe to zero if
        // either transmission reports an error.
        result = global_esc1.sendThrottle(throttle_percent);
        result2 = global_esc2.sendThrottle(throttle_percent);
        if (result != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG, "DShot transmission failed (ESC1: %s, ESC2: %s), failsafe to zero",
                     esp_err_to_name(result), esp_err_to_name(result2));
            global_esc1.sendThrottle(0);
            global_esc2.sendThrottle(0);
        }

        if (!logged_first_throttle) {
            ESP_LOGI(TAG, "Finished first throttle frame for both motors");
            logged_first_throttle = true;
        }

        // Small delay to prevent overwhelming the ESC with commands
        vTaskDelay(pdMS_TO_TICKS(10));  // 1 tick delay, typically 1ms with default FreeRTOS config
    }

    // Note: This function will never return due to the infinite loop above
}
