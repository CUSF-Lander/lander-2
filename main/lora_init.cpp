#include "lora_init.hpp"
#include "globalvars.hpp"
#include "esp_log.h"
#include <cstdio>

static const char *TAG = "LORA";

void lora_init() {
    LoRa.setPins(5, 14, 2);

    if (!LoRa.begin(915E6)) {
        ESP_LOGE(TAG, "LoRa initialisation failed");
        return;
    }

    ESP_LOGI(TAG, "LoRa Initialised");
}

void send_lora_data(const char *data) {
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
    ESP_LOGI(TAG, "LoRa packet sent: %s", data);
}

//Formatting data testing, will need to make more efficient
void format_telemetry_data(char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size,
             "Time: %llds, Euler Count: %li, Euler: (Roll: %.2f, Pitch: %.2f, Yaw: %.2f), "
             "Velocity: (x: %.2f, y: %.2f, z: %.2f), Gravity: (x: %.2f, y: %.2f, z: %.2f), "
             "Angular Accel: (x: %.2f, y: %.2f, z: %.2f), Linear Accel: (x: %.2f, y: %.2f, z: %.2f), Free Heap: %u", 
             (latest_timestamp / 1000000), euler_counter, latest_euler_data.x, latest_euler_data.y, latest_euler_data.z,
             latest_velocity_data.x, latest_velocity_data.y, latest_velocity_data.z,
             latest_gravity_data.x, latest_gravity_data.y, latest_gravity_data.z,
             latest_ang_accel_data.x, latest_ang_accel_data.y, latest_ang_accel_data.z,
             latest_lin_accel_data.x, latest_lin_accel_data.y, latest_lin_accel_data.z, esp_get_free_heap_size());
}

void send_telemetry_data() {
    char buffer[256];
    format_telemetry_data(buffer, sizeof(buffer));

    ESP_LOGI(TAG, "Sending Telemetry: %s", buffer);
    send_lora_data(buffer);
}
