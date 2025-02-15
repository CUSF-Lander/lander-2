// globalvars.cpp
#include "globalvars.hpp"

// Initialize global variables

double target_velocity_multiplier = 1.0; // Default value, can be adjusted as needed

bno08x_euler_angle_t latest_euler_data;
bno08x_gyro_t latest_velocity_data;
bno08x_accel_t latest_gravity_data;
bno08x_accel_t latest_ang_accel_data;
bno08x_accel_t latest_lin_accel_data;
int64_t latest_timestamp = 0; // Initialize timestamp to 0