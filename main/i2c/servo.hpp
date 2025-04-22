//// filepath: /Users/clementtong/Desktop/Programming2/Lander-FlightComputer-1/lander-2/main/imu_init.hpp
#ifndef LANDER_SERVO
#define LANDER_SERVO

// Initializes the IMU, registers callbacks, and creates the data rate task.
esp_err_t pca9685_set_pwm_freq(float freq);
esp_err_t pca9685_init();
esp_err_t pca9685_set_servo_angle(uint8_t servo_num, float angle);
esp_err_t pca9685_write8(uint8_t reg, uint8_t data);
esp_err_t pca9685_read8(uint8_t reg, uint8_t *data);
esp_err_t pca9685_set_pwm(uint8_t servo_num, uint16_t on, uint16_t off);

#endif // IMU_INIT_HPP