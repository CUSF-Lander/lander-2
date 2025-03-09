#include "servo.hpp"
#include <stdio.h>
#include "esp_mac.h"


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

static const char *TAG = "PCA9685_SERVO";

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        //.master.clk_speed = I2C_MASTER_FREQ_HZ,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        },
    }; 
    /*i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,          // Optional, you can use I2C_MASTER_REF_CLK_XXX
};*/

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void pca9685_write_byte(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, write_buf, 2, 1000 / portTICK_PERIOD_MS);
}

void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t write_buf[5] = {
        /*0x06 + 4 * channel,  // LEDn_ON_L register address
        on & 0xFF,           // Low byte of ON value
        on >> 8,             // High byte of ON value
        off & 0xFF,          // Low byte of OFF value
        off >> 8             // High byte of OFF value*/
        static_cast<uint8_t>(0x06 + 4 * channel),  // LEDn_ON_L register address
        static_cast<uint8_t>(on & 0xFF),           // Low byte of ON value
        static_cast<uint8_t>(on >> 8),             // High byte of ON value
        static_cast<uint8_t>(off & 0xFF),          // Low byte of OFF value
        static_cast<uint8_t>(off >> 8)     
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, write_buf, 5, 1000 / portTICK_PERIOD_MS);
}

void pca9685_init() {
    pca9685_write_byte(PCA9685_MODE1, 0x10);  // Put PCA9685 into sleep mode
    uint8_t prescale = (uint8_t)(25000000 / (4096 * 50) - 1);  // Set PWM frequency to 50 Hz
    pca9685_write_byte(PCA9685_PRESCALE, prescale);
    pca9685_write_byte(PCA9685_MODE1, 0xA0);  // Restart and set to normal mode
}

void servo_init() {
    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_master_init();
    
    ESP_LOGI(TAG, "Initializing PCA9685...");
    pca9685_init();

    vTaskDelay(250 / portTICK_PERIOD_MS);

    //todo: for testing only - servo periodic updates should not be inside servo init and there should be no infinite loop here, otherwise the rest of main.cpp will not run

    while (1) {
        // Sweep the servo from minimum to maximum
        /*for (int pulse = SERVO_MIN; pulse <= SERVO_MAX; pulse++) {
            pca9685_set_pwm(CHANNEL_0, 0, pulse);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Sweep the servo back from maximum to minimum
        for (int pulse = SERVO_MAX; pulse >= SERVO_MIN; pulse--) {
            pca9685_set_pwm(CHANNEL_0, 0, pulse);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }*/

        pca9685_set_pwm(CHANNEL_0, 0, 270);
        ESP_LOGI(TAG, "servo direction 1");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        pca9685_set_pwm(CHANNEL_0, 0, 320);
        ESP_LOGI(TAG, "servo direction 2");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}
