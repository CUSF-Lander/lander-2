#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include "../config.hpp" // Includes I2C configuration
#include "bmp390.hpp"     // Include the header file we will create
#include <cmath> // for logf

static const char *TAG = "BMP390";

// BMP390 Default I2C Address
#define BMP390_I2C_ADDR 0x77 // Or 0x76 if SDO is pulled low

// BMP390 Register Map
#define BMP390_REG_CHIP_ID      0x00
#define BMP390_REG_STATUS       0x03
#define BMP390_REG_PRESS_XLSB   0x04
#define BMP390_REG_PRESS_LSB    0x05
#define BMP390_REG_PRESS_MSB    0x06
#define BMP390_REG_TEMP_XLSB    0x07
#define BMP390_REG_TEMP_LSB     0x08
#define BMP390_REG_TEMP_MSB     0x09
#define BMP390_REG_PWR_CTRL     0x1B
#define BMP390_REG_OSR          0x1C
#define BMP390_REG_ODR          0x1D
#define BMP390_REG_CONFIG       0x1F
#define BMP390_REG_CALIB_DATA   0x31 // Start address for 21 bytes of calibration data
#define BMP390_REG_CMD          0x7E // Command register (for soft reset)

#define BMP390_CMD_SOFT_RESET   0xB6

#define BMP390_CHIP_ID_VAL      0x60 // Expected Chip ID value for BMP390

// Structure to hold calibration data
static bmp390_calib_data_t calib_data;
static bool bmp390_initialized = false;

// Helper function to write a single byte to a register
static esp_err_t bmp390_write8(uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2] = { reg, data };
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_I2C_ADDR, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed to reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// Helper function to read multiple bytes from registers
static esp_err_t bmp390_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed from reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// Helper function to read a single byte from a register
static esp_err_t bmp390_read8(uint8_t reg, uint8_t *data) {
    return bmp390_read_bytes(reg, data, 1);
}

// Read calibration data from the sensor
static esp_err_t bmp390_read_calibration_data() {
    uint8_t calib_buffer[21];
    esp_err_t ret = bmp390_read_bytes(BMP390_REG_CALIB_DATA, calib_buffer, sizeof(calib_buffer));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Parse calibration data according to BMP390 datasheet (Section 3.11.1)
    // Note: These are stored as double for precision during compensation calculations
    calib_data.par_t1 = (uint16_t)((calib_buffer[1] << 8) | calib_buffer[0]);
    calib_data.par_t2 = (uint16_t)((calib_buffer[3] << 8) | calib_buffer[2]);
    calib_data.par_t3 = (int8_t)calib_buffer[4];
    calib_data.par_p1 = (int16_t)((calib_buffer[6] << 8) | calib_buffer[5]);
    calib_data.par_p2 = (int16_t)((calib_buffer[8] << 8) | calib_buffer[7]);
    calib_data.par_p3 = (int8_t)calib_buffer[9];
    calib_data.par_p4 = (int8_t)calib_buffer[10];
    calib_data.par_p5 = (uint16_t)((calib_buffer[12] << 8) | calib_buffer[11]);
    calib_data.par_p6 = (uint16_t)((calib_buffer[14] << 8) | calib_buffer[13]);
    calib_data.par_p7 = (int8_t)calib_buffer[15];
    calib_data.par_p8 = (int8_t)calib_buffer[16];
    calib_data.par_p9 = (int16_t)((calib_buffer[18] << 8) | calib_buffer[17]);
    calib_data.par_p10 = (int8_t)calib_buffer[19];
    calib_data.par_p11 = (int8_t)calib_buffer[20];

    // Convert to floating point values needed for compensation formula (scaling factors from datasheet)
    calib_data.fp_par_t1 = (double)calib_data.par_t1 / pow(2, -8);  // T1 / 2^-8
    calib_data.fp_par_t2 = (double)calib_data.par_t2 / pow(2, 30);   // T2 / 2^30
    calib_data.fp_par_t3 = (double)calib_data.par_t3 / pow(2, 48);   // T3 / 2^48
    calib_data.fp_par_p1 = ((double)calib_data.par_p1 - pow(2, 14)) / pow(2, 20); // (P1 - 2^14) / 2^20
    calib_data.fp_par_p2 = ((double)calib_data.par_p2 - pow(2, 14)) / pow(2, 29); // (P2 - 2^14) / 2^29
    calib_data.fp_par_p3 = (double)calib_data.par_p3 / pow(2, 32);   // P3 / 2^32
    calib_data.fp_par_p4 = (double)calib_data.par_p4 / pow(2, 37);   // P4 / 2^37
    calib_data.fp_par_p5 = (double)calib_data.par_p5 / pow(2, -3);   // P5 / 2^-3
    calib_data.fp_par_p6 = (double)calib_data.par_p6 / pow(2, 6);    // P6 / 2^6
    calib_data.fp_par_p7 = (double)calib_data.par_p7 / pow(2, 8);    // P7 / 2^8
    calib_data.fp_par_p8 = (double)calib_data.par_p8 / pow(2, 15);   // P8 / 2^15
    calib_data.fp_par_p9 = (double)calib_data.par_p9 / pow(2, 48);   // P9 / 2^48
    calib_data.fp_par_p10 = (double)calib_data.par_p10 / pow(2, 48); // P10 / 2^48
    calib_data.fp_par_p11 = (double)calib_data.par_p11 / pow(2, 65); // P11 / 2^65

    ESP_LOGI(TAG, "Calibration data read successfully.");
    return ESP_OK;
}

// Compensate raw temperature value (output in degrees Celsius)
// Formula from BMP390 Datasheet Section 9.1
static double bmp390_compensate_temp(int32_t uncomp_temp) {
    double partial_data1;
    double partial_data2;

    partial_data1 = (double)(uncomp_temp - calib_data.fp_par_t1);
    partial_data2 = (double)(partial_data1 * calib_data.fp_par_t2);
    // Update the compensated temperature in calib_data since it's used for pressure compensation
    calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.fp_par_t3;

    return calib_data.t_lin; // Temperature in DegC
}

// Compensate raw pressure value (output in Pascal)
// Formula from BMP390 Datasheet Section 9.2
static double bmp390_compensate_press(int32_t uncomp_press) {
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;
    double comp_press;

    partial_data1 = calib_data.fp_par_p6 * calib_data.t_lin;
    partial_data2 = calib_data.fp_par_p7 * (calib_data.t_lin * calib_data.t_lin);
    partial_data3 = calib_data.fp_par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
    partial_out1 = calib_data.fp_par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.fp_par_p2 * calib_data.t_lin;
    partial_data2 = calib_data.fp_par_p3 * (calib_data.t_lin * calib_data.t_lin);
    partial_data3 = calib_data.fp_par_p4 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
    partial_out2 = (double)uncomp_press * (calib_data.fp_par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (double)uncomp_press * (double)uncomp_press;
    partial_data2 = calib_data.fp_par_p9 + calib_data.fp_par_p10 * calib_data.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * calib_data.fp_par_p11;

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press; // Pressure in Pa
}


// Initialize the BMP390 sensor
esp_err_t bmp390_init() {
    esp_err_t ret;

    // 1. Check Chip ID
    uint8_t chip_id = 0;
    ret = bmp390_read8(BMP390_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
    if (chip_id != BMP390_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Incorrect Chip ID: expected 0x%02X, got 0x%02X", BMP390_CHIP_ID_VAL, chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP390 Chip ID OK (0x%02X)", chip_id);

    // 2. Soft Reset
    ret = bmp390_write8(BMP390_REG_CMD, BMP390_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send soft reset command: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Wait for reset to complete

    // 3. Read Calibration Data
    ret = bmp390_read_calibration_data();
    if (ret != ESP_OK) {
        return ret;
    }

    // 4. Configure Sensor Settings
    // Set Oversampling: Pressure x8, Temperature x1 (Recommended for low power)
    // OSR[5:3] = Temp OSR, OSR[2:0] = Press OSR
    // Temp x1 = 000, Press x8 = 011 -> 0b000011 = 0x03
    uint8_t osr_config = 0x03; // Temp x1, Press x8
    ret = bmp390_write8(BMP390_REG_OSR, osr_config);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set OSR: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set Output Data Rate (ODR): 50 Hz (ODR = 0x03)
    // Check datasheet Table 16 for ODR selection
    uint8_t odr_config = 0x03; // 50 Hz
    ret = bmp390_write8(BMP390_REG_ODR, odr_config);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ODR: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set IIR Filter Coefficient: Coeff 3 (CONFIG[3:1])
    // Check datasheet Table 18 for filter coefficients
    uint8_t iir_config = (0x01 << 1); // Coeff 1 (adjust as needed, e.g., (0x02 << 1) for Coeff 3)
    ret = bmp390_write8(BMP390_REG_CONFIG, iir_config);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set IIR filter: %s", esp_err_to_name(ret));
        return ret;
    }

    // 5. Enable Pressure, Temperature, and set Normal Mode
    // PWR_CTRL[1:0] = Mode (00=Sleep, 01/10=Forced, 11=Normal)
    // PWR_CTRL[4] = Temp Enable (1=enabled)
    // PWR_CTRL[5] = Press Enable (1=enabled)
    uint8_t pwr_ctrl_config = 0b00110011; // Press En, Temp En, Normal Mode
    ret = bmp390_write8(BMP390_REG_PWR_CTRL, pwr_ctrl_config);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWR_CTRL: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMP390 Initialized Successfully.");
    bmp390_initialized = true;
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow settings to stabilize

    return ESP_OK;
}

// Calculate height (meters) using isothermal model
double calculateAltitude(double pressure, double temperatureCelsius) {
    // Constants
    constexpr double SEA_LEVEL_PRESSURE = 101325.0f; // Pa
    constexpr double CONST = 29.271267f;             // Precomputed R/(g*M)
    double temperatureKelvin = temperatureCelsius + 273.15f;
    return CONST * temperatureKelvin * logf(SEA_LEVEL_PRESSURE / pressure);
}

// Read compensated temperature and pressure data
esp_err_t bmp390_get_data(double *temperature, double *pressure, double *altitude) {
    if (!bmp390_initialized) {
        ESP_LOGE(TAG, "BMP390 not initialized.");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t data_buffer[6];

    // Read pressure and temperature data registers (6 bytes starting from PRESS_XLSB)
    ret = bmp390_read_bytes(BMP390_REG_PRESS_XLSB, data_buffer, sizeof(data_buffer));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Combine bytes into raw integer values (24-bit)
    int32_t raw_press = (int32_t)(((uint32_t)data_buffer[2] << 16) | ((uint32_t)data_buffer[1] << 8) | (uint32_t)data_buffer[0]);
    int32_t raw_temp = (int32_t)(((uint32_t)data_buffer[5] << 16) | ((uint32_t)data_buffer[4] << 8) | (uint32_t)data_buffer[3]);

    // Compensate raw values
    *temperature = bmp390_compensate_temp(raw_temp);
    *pressure = bmp390_compensate_press(raw_press);
    *altitude = calculateAltitude(*pressure, *temperature); // Calculate altitude

    return ESP_OK;
}