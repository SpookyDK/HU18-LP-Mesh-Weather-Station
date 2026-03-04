#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "moisture-soil";

#define MOISTURE_ADC_CHANNEL ADC_CHANNEL_2 // This is GPIO 3, on the ESP32-H2
#define MOISTURE_ADC_ATTEN ADC_ATTEN_DB_12

// Calibration values - measure once, change and forget
#define ADC_DRY_VALUE 3560 // Dry reference achieved by placing sensor in Very Dry dirt
#define ADC_WET_VALUE 2500 // Wet reference achieved by placing sensor in dirt fully saturated by water

// Convert raw ADC value to percentage (0% = dry, 100% = wet)
static inline uint8_t raw_to_percent(int32_t raw) {
    if (raw >= ADC_DRY_VALUE) {
        ESP_LOGW(TAG, "The measured dryness reached or exceeded expected values, Expected=%d, Got=%d", ADC_DRY_VALUE,
                 raw);
        return 0;
    }
    if (raw <= ADC_WET_VALUE) {
        ESP_LOGW(TAG, "The measured wetness is equal to or lower than expected values, Expected=%d, Got=%d",
                 ADC_WET_VALUE, raw);
        return 100;
    }
    return (uint8_t)((ADC_DRY_VALUE - raw) * 100 / (ADC_DRY_VALUE - ADC_WET_VALUE));
}

void moist_task() {
    // Initialize ADC for soil moisture sensor
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {.unit_id = ADC_UNIT_1};
    adc_oneshot_chan_cfg_t adc_chan_cfg = {.atten = MOISTURE_ADC_ATTEN, .bitwidth = ADC_BITWIDTH_12};

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOISTURE_ADC_CHANNEL, &adc_chan_cfg));

    // Initialize ADC calibration
    adc_cali_handle_t cali_handle;
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

    ESP_LOGI(TAG, "Moisture sensor ready");

    int raw;
    int voltage;
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, MOISTURE_ADC_CHANNEL, &raw));
        adc_cali_raw_to_voltage(cali_handle, raw, &voltage);

        ESP_LOGI(TAG, "Raw: %4d | Voltage: %4d mV | Moisture: %3d%%", raw, voltage, raw_to_percent(raw));
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    ESP_LOGW(TAG, "Leaving task");
}
