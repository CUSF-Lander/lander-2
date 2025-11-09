// globalvars.cpp
#include "globalvars.hpp"

// Initialize global variables

double target_velocity_multiplier = 1.0; // Default value, can be adjusted as needed

bno08x_euler_angle_t latest_euler_data;
bno08x_gyro_t latest_ang_velocity_data;
bno08x_accel_t latest_gravity_data;
bno08x_accel_t latest_ang_accel_data;
bno08x_accel_t latest_lin_accel_data;
bno08x_magf_t latest_mag_cal_quality;
latest_position_t latest_position = {0.0, 0.0, 0.0};
latest_lin_velocity_t latest_velocity = {0.0, 0.0, 0.0}; // Initialize velocity to 0
int64_t latest_timestamp = 0; // Initialize timestamp to 0
int32_t euler_counter = 0; // Initialize counter to 0
double temperature;
double pressure;
double altitude;