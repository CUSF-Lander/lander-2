#ifndef MOTOR_INIT_HPP
#define MOTOR_INIT_HPP

#include "driver/gpio.h"
#include "driver/rmt.h"

// Function to initialize the motor
void initializeMotor(gpio_num_t dshot_gpio, rmt_channel_t rmt_channel);

#endif // MOTOR_INIT_HPP