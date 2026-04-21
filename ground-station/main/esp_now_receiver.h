#ifndef ESP_NOW_RECEIVER_H
#define ESP_NOW_RECEIVER_H

#include "esp_now.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Advanced ESP-NOW receive callback with enhanced features
 * 
 * @param recv_info Information about the received packet
 * @param data Pointer to received data
 * @param len Length of received data
 */
void esp_now_advanced_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

/**
 * @brief Reset receiver statistics
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_now_receiver_stats_reset(void);

/**
 * @brief Print current receiver statistics
 */
void esp_now_receiver_print_stats(void);

/**
 * @brief Send ESTOP command to the lander
 */
void esp_now_send_estop(void);

/**
 * @brief Send ZERO_IMU command to the lander
 */
void esp_now_send_zero_imu(void);

/**
 * @brief Initialise the ESTOP task (HW button and Serial input)
 */
void init_estop_task(void);

#ifdef __cplusplus
}
#endif

#endif // ESP_NOW_RECEIVER_H