#ifndef LORA_H
#define LORA_H

#include "driver/gpio.h"
#include <cstdint>

typedef struct {
    uint32_t frequency;
    uint8_t spreading_factor;  //6 to 12
    uint8_t coding_rate;       //5 to 8 (4/5 to 4/8)
    uint8_t bandwidth;         //Bandwidth index 
} lora_config_t;

void spi_init();
void lora_reset();
void lora_config(const lora_config_t* config);
void lora_send_imu_data();
void lora_send(const char* data);
int lora_receive(char* buffer, size_t max_len);
uint8_t lora_read_reg(uint8_t reg);
void lora_write_reg(uint8_t reg, uint8_t value);

#endif