#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
 
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      22            // ESP32 Feather V2: SDA on GPIO22
#define I2C_MASTER_SCL_IO      20            // ESP32 Feather V2: SCL on GPIO20
#define I2C_MASTER_FREQ_HZ     400000
#define PCA9685_ADDR           0x40          // Default I2C address for PCA9685
 
static const char *TAG = "PCA9685_SETUP";
 
 
// Initialize the I2C master peripheral
esp_err_t i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;   // Ensure pull-ups are enabled (or use external resistors)
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
 
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
 
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
    }
    return err;
}
 
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
 
void testTest() {
    // Enable power to the STEMMA QT connector (I2C power)
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);
 
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");
 
    // Reset the PCA9685 to start from a known state
    err = pca9685_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Failed to reset PCA9685");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
 
    // Set a default PWM frequency (e.g., 1000 Hz)
    err = pca9685_set_pwm_freq(1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM frequency");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 configured successfully");
 
    while(1){
        pca9685_set_pwm_freq(10);
        vTaskDelay(pdMS_TO_TICKS(500));
 
        pca9685_set_pwm_freq(1000);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}