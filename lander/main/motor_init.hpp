#ifndef MOTOR_INIT_HPP
#define MOTOR_INIT_HPP

#include "driver/gpio.h"
#include <stdint.h>

// Temporary motor wiring test mode.
// When enabled, app_main starts only the motor task, and the motor task ignores
// ground-station ESTOP/heartbeat state and commands a fixed 5% throttle. Keep
// the mode disabled for normal builds; enable it only for a controlled bench
// test with the vehicle restrained and the propeller area clear.
#define MOTOR_WIRING_TEST_MODE 0
#define MOTOR_WIRING_TEST_THROTTLE_PERCENT 5

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

#endif // MOTOR_INIT_HPP
