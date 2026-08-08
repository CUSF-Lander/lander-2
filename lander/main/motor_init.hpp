#ifndef MOTOR_INIT_HPP
#define MOTOR_INIT_HPP

#include "driver/gpio.h"
#include "driver/rmt.h"

// Temporary motor wiring test mode.
// When enabled, app_main starts only the motor task, and the motor task ignores
// ground-station ESTOP/heartbeat state. Disable before normal flight tests.
#define MOTOR_WIRING_TEST_MODE 0
#define MOTOR_WIRING_TEST_THROTTLE_PERCENT 0
#define MOTOR_WIRING_TEST_ZERO_HOLD_MS 5000

// The motor leads are currently wired for the required physical directions, so
// startup reversal is disabled. Keep this switch and the command code for bench
// configurations that need temporary DShot command 21 after every power-up.
#define MOTOR_REVERSE_BOTH_ON_STARTUP 0
#define MOTOR_DIRECTION_COMMAND_REPEATS 10
#define MOTOR_DIRECTION_COMMAND_ZERO_HOLD_MS 300
#define MOTOR_DIRECTION_COMMAND_SETTLE_MS 500

// Function to initialize the motor
void init_2_motors(void* pvParameters);
//void init_2_motors();
//void initializeMotor(gpio_num_t dshot_gpio, rmt_channel_t rmt_channel);

//void initializeMotor(void* pvParameters);
#endif // MOTOR_INIT_HPP
