//lora_init.h

#ifndef LORA_H
#define LORA_H

#include "driver/gpio.h"

void spi_init();
void lora_reset();
void lora_config();
lora_send_imu_data();
void lora_send(const char *data);
uint8_t lora_read_reg(uint8_t reg);
void lora_write_reg(uint8_t reg, uint8_t value);

#endif // LORA_H
