#ifndef LORA_INIT_HPP
#define LORA_INIT_HPP

#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "LoRa.hpp"

void lora_init();
void send_lora_data(const char* data);
void send_telemetry_data();
void format_telemetry_data(char* buffer, size_t buffer_size);

#endif
