#ifndef ESPNOW_INIT_HPP
#define ESPNOW_INIT_HPP

#include <cstdint>

//ESP-NOW data structure for sensor telemetry
typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t timestamp;
    
    //Euler Angles
    float roll;
    float pitch;
    float yaw;
    
    //Linear Acceleration
    float lin_acc_x;
    float lin_acc_y;
    float lin_acc_z;
    
    //Position
    float pos_x;
    float pos_y;
    float pos_z;
    
    //Environment
    float pressure;

} esp_now_data_t;

void init_espnow_sender();
void esp_now_send_data();

#endif
