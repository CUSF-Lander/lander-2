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
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;   // Ensure pull-ups are enabled (or use external resistors)
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

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