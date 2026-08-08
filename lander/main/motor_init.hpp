#ifndef MOTOR_INIT_HPP
#define MOTOR_INIT_HPP

#include "driver/gpio.h"
#include <stdint.h>

// Temporary combined motor/servo bench-test mode. When enabled, app_main
// centres both TVC servos, arms both ESCs at zero, commands a fixed 5%
// throttle, then sweeps the servos around their configured midpoints. Normal
// sensor, control, ground-station ESTOP, and heartbeat handling are bypassed.
// Keep this disabled for normal builds; enable it only for a controlled bench
// test with the vehicle restrained and the propeller area clear.
#define MOTOR_WIRING_TEST_MODE 0
#define MOTOR_WIRING_TEST_THROTTLE_PERCENT 5
#define MOTOR_WIRING_TEST_SERVO_AMPLITUDE_DEG 10
#define MOTOR_WIRING_TEST_SERVO_1_PERIOD_MS 7000
#define MOTOR_WIRING_TEST_SERVO_2_PERIOD_MS 10000
#define MOTOR_WIRING_TEST_SERVO_CENTER_SETTLE_MS 1000

// First tethered-flight configuration: the ground station commands identical
// collective throttle to both motors while the hover controller commands only
// the TVC servos. Altitude and calculated motor-speed outputs are intentionally
// excluded until an omega-to-DShot/thrust calibration is available.
#define MANUAL_THROTTLE_ATTITUDE_ONLY_MODE 1

// Temporary zero-thrust hardware validation mode. ARM captures the current
// attitude, then roll displacement maps 1:1 to gimbal 1 and pitch displacement
// maps 1:1 to gimbal 2 using the model's corrective direction convention.
// Ground-station throttle commands are rejected and both ESCs remain at zero.
// Disable this after verifying the physical IMU-to-gimbal directions so the
// normal hover LQR and force-to-gimbal mapping run again.
#define GIMBAL_ATTITUDE_MAPPING_TEST_MODE 0

#if GIMBAL_ATTITUDE_MAPPING_TEST_MODE && MOTOR_WIRING_TEST_MODE
#error "GIMBAL_ATTITUDE_MAPPING_TEST_MODE and MOTOR_WIRING_TEST_MODE cannot both be enabled"
#endif

// BLHeli_S ESCs need about five seconds of valid zero-throttle DShot frames
// before they accept throttle commands.
#define MOTOR_ARM_ZERO_HOLD_MS 5000

// The motor leads are currently wired for the required physical directions, so
// startup reversal is disabled. Keep this switch and the command code for bench
// configurations that need temporary DShot command 21 after every power-up.
#define MOTOR_REVERSE_BOTH_ON_STARTUP 0
#define MOTOR_DIRECTION_COMMAND_REPEATS 10
#define MOTOR_DIRECTION_COMMAND_ZERO_HOLD_MS 300
#define MOTOR_DIRECTION_COMMAND_SETTLE_MS 500

// Function to initialize the motor
void init_2_motors(void* pvParameters);
void reinit_motor_pin(uint8_t motor_idx, gpio_num_t new_pin);
void request_motor_pin_change(uint8_t motor_idx, gpio_num_t new_pin);
bool motor_wiring_test_throttle_active();
void abort_motor_wiring_test();

#endif // MOTOR_INIT_HPP
