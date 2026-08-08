// globalvars.cpp
#include "globalvars.hpp"
#include <cmath>

// Initialize global variables
portMUX_TYPE global_spinlock = portMUX_INITIALIZER_UNLOCKED;

double target_velocity_multiplier = 1.0; // Default value, can be adjusted as needed

// Physical TVC servo calibration. The PCA9685 servo API consumes degrees,
// while U_hov.alpha1/alpha2 remain zero-centred gimbal deflections in radians.
const float servo_midpoint_1_deg = 110.0f;
const float servo_midpoint_2_deg = 80.0f;

// Convert controller-positive gimbal deflections into each servo's electrical
// direction. Both installed servos require inversion around their midpoints.
const float servo_direction_1 = -1.0f;
const float servo_direction_2 = -1.0f;

// Conservative initial travel limits around the mechanical midpoints. Servo 1
// was measured at +/-13 degrees; verify servo 2 and both linkage directions on
// hardware before increasing either limit.
const float servo_max_deflection_1_deg = 13.0f;
const float servo_max_deflection_2_deg = 13.0f;

bno08x_euler_angle_t latest_euler_data;
bno08x_gyro_t latest_ang_velocity_data;
bno08x_accel_t latest_gravity_data;
bno08x_accel_t latest_ang_accel_data;
bno08x_accel_t latest_lin_accel_data;
bno08x_magf_t latest_mag_cal_quality;
latest_position_t latest_position = {0.0, 0.0, 0.0};
latest_lin_velocity_t latest_velocity = {0.0, 0.0, 0.0}; // Initialize velocity to 0
gps_position_t latest_gps_position = {0.0, 0.0, 0.0};
u_pos_t U_pos = {0.0f, 0.0f};
u_hov_t U_hov = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
attitude_reference_t attitude_reference = {0.0f, 0.0f, 0.0f};
std::atomic<uint32_t> flight_state_reset_generation{0};
int64_t latest_timestamp = 0; // Initialize timestamp to 0
int32_t euler_counter = 0; // Initialize counter to 0
double temperature;
double pressure;
double altitude;
// ESTOP engaged on startup (safety default): motors stay at zero until the
// ground station sends the ARM command (command 4).
std::atomic<bool> estop_triggered{true};
std::atomic<bool> servo_testing_mode{false};
std::atomic<int64_t> last_gs_msg_time{0};
// Power always starts at zero. Arming must never reuse a nonzero value left
// over from a previous ground-station session.
std::atomic<uint8_t> motor_power_percent{0};

uint32_t capture_flight_reference_and_request_reset()
{
    constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;

    portENTER_CRITICAL(&global_spinlock);
    attitude_reference.roll = latest_euler_data.x * DEG_TO_RAD;
    attitude_reference.pitch = latest_euler_data.y * DEG_TO_RAD;
    attitude_reference.yaw = latest_euler_data.z * DEG_TO_RAD;
    latest_position = {0.0, 0.0, 0.0};
    latest_velocity = {0.0, 0.0, 0.0};
    altitude = 0.0;
    U_pos = {0.0f, 0.0f};
    U_hov = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    portEXIT_CRITICAL(&global_spinlock);

    // Increment only after the shared reset values have been published. Each
    // control/estimation task observes this generation and clears its private
    // state before calculating another output.
    return flight_state_reset_generation.fetch_add(1) + 1;
}
