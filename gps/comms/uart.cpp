#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "Parser.h"

namespace Uart
{

const int RX_BUF_SIZE = 1024;


#define UART_NUM UART_NUM_2
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
//com2 specified as output from um982

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    Parser parser;
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            parser.parse(data, rxBytes);
        }
    }
    free(data);
}

// Renamed from tx_task to send_command for clarity
void send_command(const char* command)
{
    static const char *TX_TAG = "TX_CMD";
    
    // Format the message with proper line terminator \r\n
    // UM980 uses # prefix for commands (not $)
    char message[128];
    snprintf(message, sizeof(message), "%s\r\n", command);
    ESP_LOGI(TX_TAG, "Sending command: '%s'", message);
    
    // Send the command
    sendData(TX_TAG, message);
    
    // Wait for transmission to complete
    vTaskDelay(500 / portTICK_PERIOD_MS);
}
}
// void app_main(void)
// {
//     init();
    
//     // Start RX task first to receive responses
//     xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    
//     // Small delay for UART to stabilize
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
    
//     // Try simple command first to test communication
//     // ESP_LOGI("MAIN", "Testing communication with VERSION command...");
//     // send_command("VERSION");
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
    
//     // Factory reset - use proper UM980 command format
//     ESP_LOGI("MAIN", "Sending FRESET...");
//     // send_command("FRESET");
//     // // FRESET causes the module to reboot. It needs time to come back online.
//     // vTaskDelay(50000 / portTICK_PERIOD_MS);  // Increased to 10 seconds
    
//     // // Configure Base Mode (using typical parameters)
//     // ESP_LOGI("MAIN", "Configuring MODE...");
//     // send_command("MODE BASE TIME 60 2 2 5");
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);
    
//     // send_command("CONFIG SIGNALGROUP 2");
//     // // Configure NMEA output - since we're ON COM2, configure COM1 or COM3 output
//     // // Or just enable GPGGA/GPGSV output on current port
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);
    
//     // send_command("rtcm1005 1");
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // send_command("rtcm1077 1");
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // send_command("rtcm1087 1");
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // send_command("rtcm1127 1");
//     // // Optional: Save the configuration so it persists through power cycles
//     // ESP_LOGI("MAIN", "Saving configuration...");
//     // send_command("SAVECONFIG");
//     // vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // Keep app_main running forever
//     while(1) {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }