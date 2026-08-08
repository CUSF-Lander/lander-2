#include "servo.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "../config.hpp"
#include "../globalvars.hpp"
#include <cmath>
 
static const char *TAG = "PCA9685";
 
// Write an 8-bit value to a PCA9685 register
esp_err_t pca9685_write8(uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(1000));
}
 
// Read an 8-bit value from a PCA9685 register
esp_err_t pca9685_read8(uint8_t reg, uint8_t *data) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, PCA9685_ADDR, &reg, 1, data, 1, pdMS_TO_TICKS(1000));
}
 
// Reset the PCA9685 (by writing the MODE1 register with the restart command)
esp_err_t pca9685_reset() {
    // MODE1 register is at 0x00, and the restart command is usually 0x80.
    return pca9685_write8(0x00, 0x80);
}

// Set the PWM frequency for the PCA9685
// This is the amount of PWM pulses per second
// The servos we are using should use 50Hz PWM frequency
esp_err_t pca9685_set_pwm_freq(float freq) {
    // Limit frequency to datasheet bounds.
    if (freq < 1) freq = 1;
    if (freq > 3500) freq = 3500;
 
    // Calculate prescale based on the formula:
    // prescale = round((oscillator / (4096 * freq)) - 1)
    // For PCA9685, the oscillator is typically 25 MHz.
    float prescale_val = (25000000.0 / (4096.0 * freq)) - 1;
    uint8_t prescale = (uint8_t)(prescale_val + 0.5);

    ESP_LOGI(TAG, "Setting PWM freq to %.2f Hz, prescale = %d", freq, prescale);
 
    uint8_t oldmode;
    esp_err_t err = pca9685_read8(0x00, &oldmode); // Read MODE1 register
    if (err != ESP_OK) return err;
 
    // Put device to sleep to set the prescaler
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Set sleep bit (bit 4)
    err = pca9685_write8(0x00, newmode);
    if (err != ESP_OK) return err;
 
    // Write the calculated prescale value to the PRESCALE register (0xFE)
    err = pca9685_write8(0xFE, prescale);
    if (err != ESP_OK) return err;
 
    // Wake the device by restoring the previous MODE1 settings
    err = pca9685_write8(0x00, oldmode);
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait a bit for oscillator to stabilize
 
    // Restart the device with auto-increment enabled (MODE1: restart and AI bits set)
    err = pca9685_write8(0x00, oldmode | 0xA1); // 0xA1 = 0x80 (restart) | 0x20 (auto-increment) | 0x01 (if needed)
    return err;
}

/**
 * @brief Set the PWM values for a specific servo.
 *
 * @param servo_num Servo number (0-15).
 * @param on Value for the LEDx_ON register (0-4095).
 * @param off Value for the LEDx_OFF register (0-4095).
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9685_set_pwm(uint8_t servo_num, uint16_t on, uint16_t off) {
    if (servo_num > 15) {
        ESP_LOGE(TAG, "Invalid servo number: %d", servo_num);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Setting servo %d: ON = %d, OFF = %d", servo_num, on, off);

    // Calculate the register addresses for the specified servo
    uint8_t led_on_l  = 0x06 + (4 * servo_num);
    uint8_t led_on_h  = 0x07 + (4 * servo_num);
    uint8_t led_off_l = 0x08 + (4 * servo_num);
    uint8_t led_off_h = 0x09 + (4 * servo_num);

    esp_err_t err;

    // Write the ON and OFF values to the registers
    err = pca9685_write8(led_on_l, on & 0xFF);
    if (err != ESP_OK) return err;

    err = pca9685_write8(led_on_h, (on >> 8) & 0x0F);
    if (err != ESP_OK) return err;

    err = pca9685_write8(led_off_l, off & 0xFF);
    if (err != ESP_OK) return err;

    err = pca9685_write8(led_off_h, (off >> 8) & 0x0F);
    return err;
}

esp_err_t pca9685_set_channel_off(uint8_t servo_num) {
    if (servo_num > 15) {
        ESP_LOGE(TAG, "Invalid servo number: %d", servo_num);
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t led_on_l  = 0x06 + (4 * servo_num);
    const uint8_t led_on_h  = 0x07 + (4 * servo_num);
    const uint8_t led_off_l = 0x08 + (4 * servo_num);
    const uint8_t led_off_h = 0x09 + (4 * servo_num);

    esp_err_t err = pca9685_write8(led_on_l, 0x00);
    if (err != ESP_OK) return err;

    err = pca9685_write8(led_on_h, 0x00);
    if (err != ESP_OK) return err;

    err = pca9685_write8(led_off_l, 0x00);
    if (err != ESP_OK) return err;

    // LEDn_OFF_H bit 4 is the PCA9685's dedicated full-OFF control.
    return pca9685_write8(led_off_h, 0x10);
}

/**
 * @brief Set the angle of a specific servo.
 *
 * @param servo_num Servo number (0-15).
 * @param angle Angle in degrees (0-180, or other range depending on your servo).
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9685_set_servo_angle(uint8_t servo_num, float angle) {
    if (!std::isfinite(angle) || angle < 0.0f || angle > 180.0f) {
        ESP_LOGE(TAG, "Invalid angle for servo %u: %.2f degrees",
                 servo_num, angle);
        return ESP_ERR_INVALID_ARG;
    }

    // Tested empirically:
    // Start point is between 110-120
    // End point is between 540-550
    // Servo characteristics (adjust these values for your specific servo)
    const float pulse_min = 110.0; // pwm min
    const float pulse_max = 550.0; // -wm max
    const float angle_min = 0.0;    // Minimum angle (degrees)
    const float angle_max = 180.0;  // Maximum angle (degrees)

    // Convert pulse width to PWM value (0-4095)
    uint16_t pwm_value = (uint16_t)(((angle-angle_min) / (angle_max-angle_min) * (pulse_max - pulse_min)) + pulse_min ); // Assuming 50Hz PWM frequency

    // Set the PWM value for the servo
    return pca9685_set_pwm(servo_num, 0, pwm_value);
}

static float clamp_float(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

esp_err_t tvc_servos_set_deflection_rad(float alpha1_rad, float alpha2_rad) {
    if (!std::isfinite(alpha1_rad) || !std::isfinite(alpha2_rad)) {
        ESP_LOGE(TAG, "Rejecting non-finite TVC command (alpha1=%.4f, alpha2=%.4f)",
                 alpha1_rad, alpha2_rad);
        return ESP_ERR_INVALID_ARG;
    }

    constexpr float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);

    const float alpha1_deg = clamp_float(
        alpha1_rad * RAD_TO_DEG,
        -servo_max_deflection_1_deg,
        servo_max_deflection_1_deg);
    const float alpha2_deg = clamp_float(
        alpha2_rad * RAD_TO_DEG,
        -servo_max_deflection_2_deg,
        servo_max_deflection_2_deg);

    const float servo1_command_deg = clamp_float(
        servo_midpoint_1_deg + servo_direction_1 * alpha1_deg, 0.0f, 180.0f);
    const float servo2_command_deg = clamp_float(
        servo_midpoint_2_deg + servo_direction_2 * alpha2_deg, 0.0f, 180.0f);

    esp_err_t result = pca9685_set_servo_angle(
        TVC_SERVO_1_CHANNEL, servo1_command_deg);
    if (result != ESP_OK) return result;

    return pca9685_set_servo_angle(
        TVC_SERVO_2_CHANNEL, servo2_command_deg);
}

esp_err_t tvc_servo_outputs_disable() {
    const esp_err_t result1 = pca9685_set_channel_off(TVC_SERVO_1_CHANNEL);
    const esp_err_t result2 = pca9685_set_channel_off(TVC_SERVO_2_CHANNEL);
    if (result1 != ESP_OK || result2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable TVC servo outputs (servo1: %s, servo2: %s)",
                 esp_err_to_name(result1), esp_err_to_name(result2));
        return result1 != ESP_OK ? result1 : result2;
    }
    return ESP_OK;
}

void tvc_servo_actuation_task(void *pvParameters) {
    constexpr TickType_t UPDATE_INTERVAL = pdMS_TO_TICKS(20); // 50 Hz
    bool outputs_disabled = false;

    ESP_LOGI(TAG,
             "TVC servo task ready: channel %d midpoint %.1f deg, channel %d midpoint %.1f deg",
             TVC_SERVO_1_CHANNEL, servo_midpoint_1_deg,
             TVC_SERVO_2_CHANNEL, servo_midpoint_2_deg);

    while (true) {
        if (estop_triggered.load()) {
            if (!outputs_disabled) {
                if (tvc_servo_outputs_disable() == ESP_OK) {
                    outputs_disabled = true;
                    ESP_LOGI(TAG, "TVC servo outputs disabled by ESTOP");
                }
            }
            vTaskDelay(UPDATE_INTERVAL);
            continue;
        }

        float alpha1_rad;
        float alpha2_rad;
        portENTER_CRITICAL(&global_spinlock);
        alpha1_rad = U_hov.alpha1;
        alpha2_rad = U_hov.alpha2;
        portEXIT_CRITICAL(&global_spinlock);

        const esp_err_t result =
            tvc_servos_set_deflection_rad(alpha1_rad, alpha2_rad);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "TVC servo command failed: %s; latching ESTOP",
                     esp_err_to_name(result));
            motor_power_percent = 0;
            estop_triggered = true;
            outputs_disabled = (tvc_servo_outputs_disable() == ESP_OK);
            vTaskDelay(UPDATE_INTERVAL);
            continue;
        }

        if (outputs_disabled) {
            ESP_LOGI(TAG, "TVC servo actuation enabled");
            outputs_disabled = false;
        }

        vTaskDelay(UPDATE_INTERVAL);
    }
}

esp_err_t pca9685_init(){
    esp_err_t err;

    // // DO NOT TOUCH THIS - this is setup done by arduino pca9685 library that isnt done in espidf
    err = pca9685_write8(0x00, 0x20); // MODE 1 Register
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pca9865_write_config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Reset the PCA9685 to start from a known state
    err = pca9685_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Failed to reset PCA9685");
        return err;
    }
    
    // // Set PWM to 0 for all channels to ensure a defined initial state
    // for (int servo_num = 0; servo_num < 16; servo_num++) {
    //     err = pca9685_set_pwm(servo_num, 0, 0);
    //     if (err != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to set initial PWM for servo %d", servo_num);
    //         return err;
    //     }
    // }

    
    err = pca9685_write8(0x00, 0x20); // MODE 1 Register
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCA9685 auto-increment: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = pca9685_set_pwm_freq(50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCA9685 PWM frequency: %s",
                 esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
