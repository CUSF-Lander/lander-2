#ifndef MOTOR_INIT_HPP
#define MOTOR_INIT_HPP

#include "driver/gpio.h"
#include <stdint.h>

// ---------------------------------------------------------------------------
// Fly-test configuration
// ---------------------------------------------------------------------------

// Arms both ESCs by holding zero throttle for this long (ms). BLHeli_S ESCs
// need ~5 s of valid DShot frames before they accept throttle commands.
#define MOTOR_ARM_ZERO_HOLD_MS 5000

// Temporary DShot reversal (BLHeli_S command 21) applied to BOTH motors at
// every startup for the counter-rotating prop stack. Command 21 is temporary:
// it must be re-sent after every power-up. Set to 1 ONLY if the wiring/props
// actually require software reversal — otherwise the motors will spin the
// wrong way.
#define MOTOR_REVERSE_BOTH_ON_STARTUP 0
#define MOTOR_DIRECTION_COMMAND_REPEATS 10
#define MOTOR_DIRECTION_COMMAND_ZERO_HOLD_MS 300
#define MOTOR_DIRECTION_COMMAND_SETTLE_MS 500

// Motor wiring test mode: bypasses IMU/GPS/ESP-NOW and drives both ESCs at a
// fixed throttle, ignoring ground-station ESTOP/heartbeat. Bench use only.
#define MOTOR_WIRING_TEST_MODE 0
#define MOTOR_WIRING_TEST_THROTTLE_PERCENT 5

// Function to initialize the motor
void init_2_motors(void* pvParameters);
//void init_2_motors();
//void initializeMotor(gpio_num_t dshot_gpio, rmt_channel_t rmt_channel);

//void initializeMotor(void* pvParameters);
void reinit_motor_pin(uint8_t motor_idx, gpio_num_t new_pin);
void request_motor_pin_change(uint8_t motor_idx, gpio_num_t new_pin);


#endif // MOTOR_INIT_HPP
