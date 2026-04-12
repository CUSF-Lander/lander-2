#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart.h"
static const char *TAG = "ROVER";

extern "C" void app_main(void)
{
	ESP_LOGI(TAG, "Rover app started");
    Uart::init();
    
    // Start RX task first to receive responses
    xTaskCreate(Uart::rx_task, "uart_rx_task", 1024 * 16, NULL, 5, NULL);
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
