#ifndef UART_H
#define UART_H

namespace Uart{
void init(void);

int sendData(const char* logName, const char* data);

void rx_task(void *arg);

void send_command(const char* command);

} // namespace Uart


#endif // uart_init_h