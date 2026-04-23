#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "moist_soil.h"
#include "packet_def.h"
#include "pin_config.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "moisture-soil";
bool moist_shared_status = true;
static bool initiated_moist = false;
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;

// Calibration values - measure once, change and forget
#define ADC_DRY_VALUE 900 // Dry reference achieved by placing sensor in Very Dry dirt
#define ADC_WET_VALUE 200 // Wet reference achieved by placing sensor in dirt fully saturated by water

// Convert raw ADC value to percentage (0% = dry, 100% = wet)
static inline uint8_t voltage_to_percent(int32_t raw) {
    if (raw >= ADC_DRY_VALUE) {
        ESP_LOGW(TAG, "The measured dryness reached or exceeded expected values, Expected=%d, Got=%d", ADC_DRY_VALUE, raw);
        moist_shared_status = true;
        return 0;
    }
    if (raw <= ADC_WET_VALUE) {
        ESP_LOGW(TAG, "The measured wetness is equal to or lower than expected values, Expected=%d, Got=%d", ADC_WET_VALUE, raw);
        moist_shared_status = true;
        return 100;
    }
    moist_shared_status = false;
    return (uint8_t)((ADC_DRY_VALUE - raw) * 100 / (ADC_DRY_VALUE - ADC_WET_VALUE));
}

static void init_moist() {
    // Initialize ADC for soil moisture sensor
    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {.unit_id = ADC_UNIT_1};
    adc_oneshot_chan_cfg_t adc_chan_cfg = {.atten = MOISTURE_ADC_ATTEN, .bitwidth = ADC_BITWIDTH_12};

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOISTURE_ADC_CHANNEL, &adc_chan_cfg));

    // Initialize ADC calibration
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = MOISTURE_ADC_CHANNEL,
        .atten = MOISTURE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGE(TAG, "ADC calibration failed, voltage readings will be wrong");
    }

    initiated_moist = true;
    ESP_LOGI(TAG, "Moisture sensor ready");
}

esp_err_t get_moist_read(sensor_payload_t *payload) {
    if (!initiated_moist) {
        ESP_LOGW(TAG, "Moist not initiated");
        init_moist();
    }
    int raw;
    int voltage;
    if (adc_oneshot_read(adc_handle, MOISTURE_ADC_CHANNEL, &raw) != ESP_OK) {
        ESP_LOGW(TAG, "SoilMoist> Failed to read");
        return ESP_FAIL;
    }
    adc_cali_raw_to_voltage(cali_handle, raw, &voltage);

    payload->soil_moisture = voltage_to_percent(voltage);
    // ESP_LOGI(TAG, "Raw: %4d | Voltage: %4d mV | Moisture: %3d%%", raw, voltage, moist_shared_percentage);
    return ESP_OK;
}
