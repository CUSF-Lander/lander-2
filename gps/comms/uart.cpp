#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "Parser.h"
#include "comm.h"
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

    // 2x RX_BUF_SIZE so a partial frame carried over from the previous read
    // never causes an overflow when new bytes are appended behind it.
    const size_t BUF_SIZE = RX_BUF_SIZE * 2;
    uint8_t* buf = (uint8_t*)malloc(BUF_SIZE);
    int buf_len = 0;

    while (1) {
        // Append freshly received bytes after any carry-over bytes.
        int n = uart_read_bytes(UART_NUM, buf + buf_len,
                                BUF_SIZE - buf_len,
                                pdMS_TO_TICKS(100));
        if (n > 0) buf_len += n;
        if (buf_len == 0) continue;

        // Scan the buffer for complete RTCM3 frames.
        // RTCM3 frame layout:
        //   byte 0      : 0xD3  (preamble)
        //   bytes 1-2   : reserved(6 bits) + payload_length(10 bits)
        //   bytes 3..N  : payload  (N = payload_length)
        //   bytes N+1..N+3 : CRC24
        int pos = 0;
        while (pos < buf_len) {

            // Scan forward to the next preamble byte.
            if (buf[pos] != 0xD3) { pos++; continue; }

            // Need at least 3 header bytes to read the length.
            if (pos + 3 > buf_len) break;

            // Extract the 10-bit payload length.
            uint16_t payload_len = ((buf[pos + 1] & 0x03) << 8) | buf[pos + 2];

            // RTCM3 payload is capped at 1023 bytes by the spec.
            // A larger value means this 0xD3 is not a real preamble.
            if (payload_len > 1023) { pos++; continue; }

            uint16_t frame_len = 3 + payload_len + 3;  // header + payload + CRC

            // Wait for more UART bytes if the frame is not yet complete.
            if (pos + frame_len > (size_t)buf_len) break;

            // Complete frame — forward it if a peer is registered.
            if (WIFI::is_peer_known()) {
                if (frame_len <= WIFI::MAX_CORRECTION_LEN) {
                    WIFI::send_correction(buf + pos, (uint8_t)frame_len);
                    ESP_LOGI(RX_TASK_TAG, "Sent RTCM frame: %u bytes", frame_len);
                } else {
                    // A single RTCM message exceeded 248 bytes — this is
                    // unusual; log and drop rather than sending a corrupt frame.
                    ESP_LOGW(RX_TASK_TAG, "RTCM frame %u bytes > MAX_CORRECTION_LEN, dropping", frame_len);
                }
            }

            pos += frame_len;
        }

        // Shift any unprocessed bytes (partial frame) to the front of the
        // buffer so they are prepended to the next UART read.
        if (pos > 0) {
            buf_len -= pos;
            if (buf_len > 0) memmove(buf, buf + pos, buf_len);
        }
    }
    free(buf);
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