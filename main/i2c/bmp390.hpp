#ifndef BMP390_HPP
#define BMP390_HPP

#include "esp_err.h"
#include <stdint.h>

// Structure to hold calibration data (internal representation)
typedef struct {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t   par_t3;
    int16_t  par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int8_t   par_p10;
    int8_t   par_p11;

    // Floating point representations for compensation
    double fp_par_t1;
    double fp_par_t2;
    double fp_par_t3;
    double fp_par_p1;
    double fp_par_p2;
    double fp_par_p3;
    double fp_par_p4;
    double fp_par_p5;
    double fp_par_p6;
    double fp_par_p7;
    double fp_par_p8;
    double fp_par_p9;
    double fp_par_p10;
    double fp_par_p11;

    // Intermediate temperature value needed for pressure compensation
    double t_lin;
} bmp390_calib_data_t;


/**
 * @brief Initialize the BMP390 sensor.
 *
 * Checks chip ID, performs a soft reset, reads calibration data,
 * and configures the sensor for normal operation (oversampling, ODR, filter, power mode).
 * Must be called before bmp390_get_data().
 * Requires I2C master to be initialized first (via i2c_master_init()).
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t bmp390_init();

/**
 * @brief Read compensated temperature and pressure data from the BMP390.
 *
 * Reads the raw sensor values and applies calibration compensation formulas.
 *
 * @param temperature Pointer to a double where the compensated temperature (in degrees Celsius) will be stored.
 * @param pressure Pointer to a double where the compensated pressure (in Pascals) will be stored.
 * @return esp_err_t ESP_OK on success, otherwise an error code (e.g., if sensor not initialized or I2C read fails).
 */
esp_err_t bmp390_get_data(double *temperature, double *pressure);


#endif // BMP390_HPP