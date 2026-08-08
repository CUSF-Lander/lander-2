#ifndef GLOBALVARS_HPP
#define GLOBALVARS_HPP

#include <atomic>
#include <cstdint>
#include "esp_log.h"
#include <vector>
#include <BNO08xGlobalTypes.hpp>
#include <freertos/FreeRTOS.h>

// Global variables
// Physical TVC servo calibration in degrees. Controller gimbal outputs remain
// zero-centred radians and are converted only by the servo actuator task.
extern const float servo_midpoint_1_deg;
extern const float servo_midpoint_2_deg;
extern const float servo_direction_1;
extern const float servo_direction_2;
extern const float servo_max_deflection_1_deg;
extern const float servo_max_deflection_2_deg;

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

typedef struct {
    double x;
    double y;
    double z;
} gps_position_t;

extern gps_position_t latest_gps_position;

// Output of the position controller: desired roll and pitch [rad] for the hover controller
typedef struct {
    float roll;   // desired roll  [rad]
    float pitch;  // desired pitch [rad]
} u_pos_t;

extern u_pos_t U_pos;

// Output of the hover controller: actuator commands computed from LQR + thrust geometry
typedef struct {
    float alpha1;  // gimbal angle 1   [rad]
    float alpha2;  // gimbal angle 2   [rad]
    float omega1;  // motor 1 speed    [rad/s]
    float omega2;  // motor 2 speed    [rad/s]
    float lambda;  // motor thrust ratio [-]
} u_hov_t;

extern u_hov_t U_hov;

// Attitude held by the attitude-only controller. A ZERO_IMU command or a
// successful ARM captures the current IMU orientation as the zero-error pose.
typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_reference_t;

extern attitude_reference_t attitude_reference;
extern std::atomic<uint32_t> flight_state_reset_generation;
uint32_t capture_flight_reference_and_request_reset();

extern std::atomic<bool> estop_triggered;
extern std::atomic<bool> servo_testing_mode;
extern std::atomic<int64_t> last_gs_msg_time;
extern std::atomic<uint8_t> motor_power_percent;

// Data storage (using vectors for dynamic storage)


#endif
