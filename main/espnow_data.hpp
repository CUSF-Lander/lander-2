#ifndef ESPNOW_DATA_HPP
#define ESPNOW_DATA_HPP

#include <cstdint>

//ESP-NOW data structure for sensor telemetry
typedef struct {
    uint32_t sequence;
    uint32_t timestamp;
    float sensor_data[10];     //40 bytes - [0-2]: euler angles, [3-5]: lin accel, [6-8]: position, [9]: pressure
    uint8_t reserved[202];     //202 bytes - reserved for future use (total: 250 bytes)
} esp_now_data_t;

#endif 
