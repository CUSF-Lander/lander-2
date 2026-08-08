#ifndef LANDER_SERVO
#define LANDER_SERVO

#include <cstdint>
#include "esp_err.h"

// Temporary, servo-only bench test. When enabled, app_main initializes only
// I2C and the PCA9685, then accepts angles for one channel over the serial
// terminal. Motors, sensors, GPS, ESP-NOW, and flight-control tasks are not
// initialized.
#define SERVO_TERMINAL_TEST_MODE 0

// Physical TVC servo connections on the PCA9685 Servo FeatherWing.
#define TVC_SERVO_1_CHANNEL 0
#define TVC_SERVO_2_CHANNEL 7
#define SERVO_TERMINAL_TEST_CHANNEL TVC_SERVO_2_CHANNEL

// Bench calibration recorded for PCA9685 channel 0:
//   midpoint: 110 degrees
//   tested travel: midpoint +/- 13 degrees (97 to 123 degrees)

// PCA9685 setup and servo-output helpers.
esp_err_t pca9685_set_pwm_freq(float freq);
esp_err_t pca9685_init();
esp_err_t pca9685_set_servo_angle(uint8_t servo_num, float angle);
esp_err_t pca9685_write8(uint8_t reg, uint8_t data);
esp_err_t pca9685_read8(uint8_t reg, uint8_t *data);
esp_err_t pca9685_set_pwm(uint8_t servo_num, uint16_t on, uint16_t off);
esp_err_t pca9685_set_channel_off(uint8_t servo_num);

#endif // LANDER_SERVO
