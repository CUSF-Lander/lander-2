#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

namespace Uart{
void init(void);

int sendData(const char* logName, const char* data);

void rx_task(void *arg);

void read_correction(void* arg);

void rx_gngga_task(void* arg);

void send_command(const char* command);

void send_correction(const uint8_t* data, size_t len);
} // namespace Uart


#endif // uart_init_h