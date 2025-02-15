#ifndef GLOBALVARS_HPP
#define GLOBALVARS_HPP

#include "esp_log.h"
#include <vector>
#include <BNO08xGlobalTypes.hpp>

// Global variables
extern bno08x_euler_angle_t latest_euler_data;
extern bno08x_gyro_t latest_velocity_data;
extern bno08x_accel_t latest_gravity_data;
extern bno08x_accel_t latest_ang_accel_data;
extern bno08x_accel_t latest_lin_accel_data;
extern int64_t latest_timestamp; // Store timestamps for each data point

// Data storage (using vectors for dynamic storage)


#endif