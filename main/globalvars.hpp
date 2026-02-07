#ifndef GLOBALVARS_HPP
#define GLOBALVARS_HPP

#include "esp_log.h"
#include <vector>
#include <BNO08xGlobalTypes.hpp>
#include <freertos/FreeRTOS.h>

// Global variables
extern portMUX_TYPE global_spinlock;
extern bno08x_euler_angle_t latest_euler_data;
extern bno08x_gyro_t latest_ang_velocity_data;
extern bno08x_accel_t latest_gravity_data;
extern bno08x_accel_t latest_ang_accel_data;
extern bno08x_accel_t latest_lin_accel_data;
extern bno08x_magf_t latest_mag_cal_quality;
extern int64_t latest_timestamp; // Store timestamps for each data point
extern int32_t euler_counter;
extern double temperature;
extern double pressure;
extern double altitude;

typedef struct {
    double x;
    double y;
    double z;
} latest_position_t;

extern latest_position_t latest_position;

typedef struct {
    double x;
    double y;
    double z;
} latest_lin_velocity_t;

extern latest_lin_velocity_t latest_velocity;

// Data storage (using vectors for dynamic storage)


#endif