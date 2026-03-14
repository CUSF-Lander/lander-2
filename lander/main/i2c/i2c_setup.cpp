#include "driver/i2c.h"
#include "esp_log.h"
#include "../config.hpp"

static const char *TAG = "I2C_SETUP";

// Initialize the I2C master peripheral
esp_err_t i2c_master_init() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = 0; // ESP-IDF 5.5.1 compatibility

  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
      return err;
  }

  // Install driver with proper buffer sizes for ESP-IDF 5.5.1
  err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
      return err;
  }
  
  // Set I2C timeout (important for ESP-IDF 5.5.1)
  err = i2c_set_timeout(I2C_MASTER_NUM, 0xFFFFF);
  if (err != ESP_OK) {
      ESP_LOGW(TAG, "i2c_set_timeout warning: %s", esp_err_to_name(err));
  }
  
  ESP_LOGI(TAG, "I2C master initialized on port %d (SDA:%d, SCL:%d, Freq:%lu Hz)",
           I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
  
  return ESP_OK;
}