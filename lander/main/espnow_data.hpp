#ifndef ESPNOW_DATA_HPP
#define ESPNOW_DATA_HPP

#include <cstdint>

//ESP-NOW data structure for sensor telemetry
typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t timestamp;
    
    //euler angles
    float roll;
    float pitch;
    float yaw;
    
    //linear acceleration
    float lin_acc_x;
    float lin_acc_y;
    float lin_acc_z;
    
    //position
    float pos_x;
    float pos_y;
    float pos_z;
    
    //environment
    float pressure;

} esp_now_data_t;

//ESP-NOW command structure for ground station to lander
typedef struct __attribute__((packed)) {
    uint8_t command;
} esp_now_cmd_t;

#endif