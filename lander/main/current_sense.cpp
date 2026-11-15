#include "current_sense.hpp"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "CURRENT_SENSE";

// ---------------------------------------------------------------------------
// ESC current sense
//
// The SpeedyBee ESC's analog current output is wired to D33 on the Adafruit
// Feather V2 (silkscreen D33 == GPIO33). The reading is logged at 1 Hz and sent
// to the ground station inside the ESP-NOW telemetry packet (current_a).
//
// Pin caveat: this must be an ADC1 pin. On the ESP32 the ADC2 pins - including
// D27 / GPIO27, which is ADC2 channel 7 - cannot be read while WiFi / ESP-NOW is
// running, because the radio owns the ADC2 hardware. A current lead on D27 reads
// nothing, so it is rejected at compile time below.
//
// Calibration: CURRENT_SENSE_A_PER_V and CURRENT_SENSE_ZERO_V describe the ESC
// output (0.1 V per amp with 0 V at 0 A is common). Measure the sense voltage at
// a known current and adjust them if the readings are off.
// ---------------------------------------------------------------------------
#define CURRENT_SENSE_PIN 33
#define CURRENT_SENSE_SAMPLE_MS 100
#define CURRENT_SENSE_A_PER_V 10.0f   // amps per volt of sense output
#define CURRENT_SENSE_ZERO_V 0.0f     // sense output at 0 A (offset sensors)

// At 12 dB attenuation the ADC tops out around 3.1-3.3 V. Anything at or above
// this is the rail, not a real current: a disconnected lead reads as a constant
// full-scale value (10 A/V * 3.3 V = 33 A in the bench logs).
#define CURRENT_SENSE_RAIL_V 3.0f

#if CURRENT_SENSE_PIN == 36
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_0
#elif CURRENT_SENSE_PIN == 37
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_1
#elif CURRENT_SENSE_PIN == 38
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_2
#elif CURRENT_SENSE_PIN == 39
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_3
#elif CURRENT_SENSE_PIN == 32
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_4
#elif CURRENT_SENSE_PIN == 33
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_5
#elif CURRENT_SENSE_PIN == 34
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_6
#elif CURRENT_SENSE_PIN == 35
#define CURRENT_SENSE_ADC_CHANNEL ADC_CHANNEL_7
#elif CURRENT_SENSE_PIN == 27
#error "GPIO 27 is an ADC2 pin - ADC2 cannot be read while WiFi/ESP-NOW is running. Use D33 (ADC1 channel 5)."
#else
#error "CURRENT_SENSE_PIN must be an ESP32 ADC1 pin (32-39)"
#endif

float latest_current_a = 0.0f;

static adc_oneshot_unit_handle_t current_adc = NULL;
static adc_cali_handle_t current_cali = NULL;

static void current_sense_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id  = ADC_UNIT_1;
    unit_cfg.clk_src  = ADC_RTC_CLK_SRC_DEFAULT;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &current_adc) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC for current sense");
        current_adc = NULL;
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(current_adc, CURRENT_SENSE_ADC_CHANNEL, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel for current sense");
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id  = ADC_UNIT_1;
    cali_cfg.chan     = CURRENT_SENSE_ADC_CHANNEL;
    cali_cfg.atten    = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &current_cali) != ESP_OK) {
        current_cali = NULL;
    }
#endif

    ESP_LOGI(TAG, "ESC current sense on GPIO %d (%.1f A/V, zero %.2f V)",
             CURRENT_SENSE_PIN, CURRENT_SENSE_A_PER_V, CURRENT_SENSE_ZERO_V);
}

static bool read_current_sense(float *volts, float *amps)
{
    if (current_adc == NULL) {
        return false;
    }

    // Average a few samples to soften ESC switching noise.
    int raw_sum = 0;
    for (int i = 0; i < 4; i++) {
        int raw = 0;
        if (adc_oneshot_read(current_adc, CURRENT_SENSE_ADC_CHANNEL, &raw) != ESP_OK) {
            return false;
        }
        raw_sum += raw;
    }

    int millivolts = 0;
    if (current_cali != NULL) {
        if (adc_cali_raw_to_voltage(current_cali, raw_sum / 4, &millivolts) != ESP_OK) {
            return false;
        }
    } else {
        millivolts = (raw_sum / 4) * 3300 / 4095;
    }

    *volts = millivolts / 1000.0f;
    if (*volts >= CURRENT_SENSE_RAIL_V) {
        // Saturated: report invalid instead of a plausible-looking current.
        *amps = CURRENT_SENSE_INVALID_A;
        return true;
    }
    *amps = (*volts - CURRENT_SENSE_ZERO_V) * CURRENT_SENSE_A_PER_V;
    return true;
}

static void current_sense_task(void *pvParameters)
{
    current_sense_adc_init();

    bool read_failed = false;
    bool rail_warned = false;
    int log_divider = 0;

    TickType_t last_wake = xTaskGetTickCount();
    while (1)
    {
        float volts = 0.0f;
        float amps = 0.0f;

        if (read_current_sense(&volts, &amps)) {
            latest_current_a = amps;
            read_failed = false;

            if (amps == CURRENT_SENSE_INVALID_A) {
                if (!rail_warned) {
                    rail_warned = true;
                    ESP_LOGW(TAG,
                             "Current sense at ADC rail (%.2f V): lead disconnected or ESC unpowered; reporting %.1f A",
                             volts, amps);
                }
            } else {
                if (rail_warned) {
                    rail_warned = false;
                    ESP_LOGI(TAG, "Current sense recovered");
                }

                // Keep the log readable at 1 Hz instead of 10 Hz.
                if (++log_divider >= 10) {
                    log_divider = 0;
                    ESP_LOGI(TAG, "ESC current: %.2f A (%.2f V sense)", amps, volts);
                }
            }
        } else if (!read_failed) {
            read_failed = true;
            ESP_LOGE(TAG, "ESC current read failed on GPIO %d", CURRENT_SENSE_PIN);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CURRENT_SENSE_SAMPLE_MS));
    }
}

void init_current_sense()
{
    xTaskCreate(current_sense_task, "current_sense", 3072, NULL, 4, NULL);
}
