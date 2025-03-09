#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"

// PCA9685 I2C address and parameters
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define SERVO_MIN 150
#define SERVO_MAX 600
#define CHANNEL_0 0

// Function declarations
/**
 * @brief Initialize I2C master
 */
void i2c_master_init();

/**
 * @brief Write a byte to PCA9685 register
 * @param reg Register address
 * @param data Data byte to write
 */
void pca9685_write_byte(uint8_t reg, uint8_t data);

/**
 * @brief Set PWM values for a channel
 * @param channel Channel number (0-15)
 * @param on ON time (0-4095)
 * @param off OFF time (0-4095)
 */
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief Initialize PCA9685 servo controller
 */
void pca9685_init();

/**
 * @brief Initialize servo system and start demo sweep
 */
void servo_init();

#endif // SERVO_HPP