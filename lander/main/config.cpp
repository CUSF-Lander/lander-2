#include <cstdint>
#include "driver/i2c.h"

int I2C_MASTER_SDA_IO       = 22;
int I2C_MASTER_SCL_IO       = 20;
uint32_t I2C_MASTER_FREQ_HZ = 100000; // Default I2C clock speed
uint8_t PCA9685_ADDR        = 0x40;
i2c_port_t I2C_MASTER_NUM   = I2C_NUM_0; // Default I2C port number