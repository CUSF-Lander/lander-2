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

// DShot reserves values 0-47 for stop and special commands. Values 48-2047
// represent the usable throttle range.
static constexpr uint16_t MIN_THROTTLE = 48;
static constexpr uint16_t MAX_THROTTLE = 2047;

// Global ESC instances allow an ESTOP-gated ground-station command to move a
// motor output to another safe GPIO during bench work.
static DShotRMT global_esc1;
static DShotRMT global_esc2;

static uint16_t power_percent_to_throttle(uint8_t power_percent)
{
    // Zero is a distinct DShot stop command, not the bottom of the 48-2047
    // throttle range.
    if (power_percent == 0) {
        return 0;
    }

    if (power_percent > 100) {
        power_percent = 100;
    }

    return MIN_THROTTLE
        + static_cast<uint16_t>(
            (power_percent * (MAX_THROTTLE - MIN_THROTTLE)) / 100);
}

// Reject pins unsafe for DShot before tearing down the old pin: 34-39
// (input-only), 6-11 (flash), 0/12/15 (strapping), 1/3 (UART0), 2 (sensor power).
static bool is_valid_dshot_pin(gpio_num_t pin)
{
    const int p = static_cast<int>(pin);

    if (p < 0 || p > 33) {
        ESP_LOGE(TAG, "Invalid DShot GPIO %d (valid: 0-33, output-capable)", p);
        return false;
    }
    if ((p >= 6 && p <= 11) || p == 0 || p == 1 || p == 2 || p == 3
        || p == 12 || p == 15) {
        ESP_LOGE(TAG,
                 "Refusing DShot GPIO %d: reserved (flash, strapping, UART0 or sensor power)",
                 p);
        return false;
    }

    return true;
}

void reinit_motor_pin(uint8_t motor_idx, gpio_num_t new_pin)
{
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

    if (!is_valid_dshot_pin(new_pin)) {
        ESP_LOGE(TAG, "Ignoring SET_PIN for motor %d: GPIO %d is not usable for DShot",
                 motor_idx, static_cast<int>(new_pin));
        return;
    }

    // Stop driving the old pin before releasing the RMT channel.
    esp_err_t result = target->sendThrottle(0);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop motor %d before changing pin: %s",
                 motor_idx, esp_err_to_name(result));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    result = target->uninstall();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall DShot driver for motor %d: %s",
                 motor_idx, esp_err_to_name(result));
        return;
    }

    result = target->install(new_pin);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install DShot driver for motor %d on GPIO %d: %s",
                 motor_idx, static_cast<int>(new_pin), esp_err_to_name(result));
        return;
    }

    // init() holds zero throttle for about five seconds, blocking this task.
    // The other ESC receives no DShot frames during that window, so runtime pin
    // changes are permitted only while ESTOP is engaged and remain bench-only.
    result = target->init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize motor %d on GPIO %d: %s",
                 motor_idx, static_cast<int>(new_pin), esp_err_to_name(result));
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
        ESP_LOGW(TAG, "Motor pin change queue full, dropping request for motor %d",
                 motor_idx);
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
        const esp_err_t result1 = global_esc1.sendThrottle(0);
        const esp_err_t result2 = global_esc2.sendThrottle(0);
        if (result1 != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send zero throttle (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(result1), esp_err_to_name(result2));
            return ESP_ERR_INVALID_STATE;
        }
        vTaskDelay(interval);
    }
    return ESP_OK;
}

void init_2_motors(void* pvParameters)
{
    // Physical motor mapping for the current counter-rotating propeller stack:
    //   ESC 1 / GPIO 4  = bottom propeller
    //   ESC 2 / GPIO 25 = top propeller
    // ESP-IDF 6 allocates RMT channels dynamically, so channel numbers are not
    // supplied by this application.
    constexpr gpio_num_t dshot_gpio1 = GPIO_NUM_4;
    constexpr gpio_num_t dshot_gpio2 = GPIO_NUM_25;

    ESP_LOGI(TAG, "Initializing DShot RMT for both motors");

    esp_err_t result1 = global_esc1.install(dshot_gpio1);
    if (result1 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ESC 1 DShot driver: %s",
                 esp_err_to_name(result1));
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t result2 = global_esc2.install(dshot_gpio2);
    if (result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ESC 2 DShot driver: %s",
                 esp_err_to_name(result2));
        ESP_LOGE(TAG, "Motor output disabled");
        global_esc1.uninstall();
        vTaskDelete(nullptr);
        return;
    }

    // Arm both ESCs together. Sequential five-second init() calls caused the
    // bottom motor to initialize about five seconds before the top motor.
    ESP_LOGI(TAG, "Holding both ESCs at zero throttle for %d ms",
             MOTOR_ARM_ZERO_HOLD_MS);
    if (hold_both_zero_ms(MOTOR_ARM_ZERO_HOLD_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "Both ESCs armed successfully");

#if MOTOR_REVERSE_BOTH_ON_STARTUP
    // BLHeli_S command 21 is a temporary reversal relative to the direction
    // saved in ESC flash. Ten packets exceed the six-packet minimum.
    ESP_LOGW(TAG, "Applying temporary DShot reversal to both motors");

    if (hold_both_zero_ms(MOTOR_DIRECTION_COMMAND_ZERO_HOLD_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(nullptr);
        return;
    }

    for (int i = 0; i < MOTOR_DIRECTION_COMMAND_REPEATS; ++i) {
        result1 = global_esc1.sendDirectionCommand(true);
        result2 = global_esc2.sendDirectionCommand(true);
        if (result1 != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send direction command (ESC1: %s, ESC2: %s)",
                     esp_err_to_name(result1), esp_err_to_name(result2));
            ESP_LOGE(TAG, "Motor output disabled");
            vTaskDelete(nullptr);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (hold_both_zero_ms(MOTOR_DIRECTION_COMMAND_SETTLE_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Motor output disabled");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG,
             "DShot reversal commands sent to both motors; "
             "direction must be verified with propellers removed");
#endif

    if (motor_pin_change_queue == nullptr) {
        motor_pin_change_queue = xQueueCreate(4, sizeof(motor_pin_change_request_t));
    }
    if (motor_pin_change_queue == nullptr) {
        ESP_LOGW(TAG, "Motor pin-change queue unavailable; runtime pin changes disabled");
    }

#if MOTOR_WIRING_TEST_MODE
    ESP_LOGW(TAG,
             "MOTOR_WIRING_TEST_MODE enabled: ignoring ground-station timeout and software ESTOP");
    ESP_LOGW(TAG, "Fixed bench-test throttle: %d%%",
             MOTOR_WIRING_TEST_THROTTLE_PERCENT);
#endif

    bool logged_first_throttle = false;
    ESP_LOGI(TAG, "Entering motor control loop");

    while (true) {
        motor_pin_change_request_t request;
        while (motor_pin_change_queue != nullptr
               && xQueueReceive(motor_pin_change_queue, &request, 0) == pdTRUE) {
            if (!estop_triggered.load()) {
                ESP_LOGW(TAG,
                         "Rejecting SET_PIN for motor %d while motors are armed",
                         request.motor_idx);
                continue;
            }
            reinit_motor_pin(request.motor_idx, request.new_pin);
        }

#if !MOTOR_WIRING_TEST_MODE
        // Trigger ESTOP if ground-station traffic disappears for 500 ms.
        if (last_gs_msg_time > 0
            && (esp_timer_get_time() - last_gs_msg_time) > 500000) {
            if (!estop_triggered.load()) {
                ESP_LOGE(TAG,
                         "Ground station connection lost (500ms timeout)! Triggering ESTOP.");
                motor_power_percent = 0;
                estop_triggered = true;
            }
        }

        if (estop_triggered.load()) {
            global_esc1.sendThrottle(0);
            global_esc2.sendThrottle(0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
#endif

#if MOTOR_WIRING_TEST_MODE
        const uint16_t throttle_value =
            power_percent_to_throttle(MOTOR_WIRING_TEST_THROTTLE_PERCENT);
#else
        const uint16_t throttle_value =
            power_percent_to_throttle(motor_power_percent.load());
#endif

        if (!logged_first_throttle) {
            ESP_LOGI(TAG, "About to send first throttle frame: throttle=%u",
                     throttle_value);
        }

        result1 = global_esc1.sendThrottle(throttle_value);
        result2 = global_esc2.sendThrottle(throttle_value);
        if (result1 != ESP_OK || result2 != ESP_OK) {
            ESP_LOGE(TAG,
                     "DShot transmission failed (ESC1: %s, ESC2: %s); latching ESTOP",
                     esp_err_to_name(result1), esp_err_to_name(result2));
            global_esc1.sendThrottle(0);
            global_esc2.sendThrottle(0);
            motor_power_percent = 0;
            estop_triggered = true;
        }

        if (!logged_first_throttle) {
            ESP_LOGI(TAG, "Finished first throttle frame for both motors");
            logged_first_throttle = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz update rate
    }
}
