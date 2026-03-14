#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include "driver/i2c.h"

extern int I2C_MASTER_SDA_IO;
extern int I2C_MASTER_SCL_IO;
extern uint32_t I2C_MASTER_FREQ_HZ;
extern uint8_t PCA9685_ADDR;
extern i2c_port_t I2C_MASTER_NUM;

#endif