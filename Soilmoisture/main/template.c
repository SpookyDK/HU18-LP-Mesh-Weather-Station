#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ADC configuration for soil moisture sensor
#define MOISTURE_ADC_CHANNEL ADC_CHANNEL_2
#define MOISTURE_ADC_ATTEN   ADC_ATTEN_DB_12
// calibartion values measure once change and forget
#define ADC_DRY_VALUE 3000
#define ADC_WET_VALUE 1000

// Convert raw ADC value to percentage (0% = dry, 100% = wet)
static int raw_to_percent(int raw)
{
    if (raw > ADC_DRY_VALUE) raw = ADC_DRY_VALUE;
    if (raw < ADC_WET_VALUE) raw = ADC_WET_VALUE;
    return (ADC_DRY_VALUE - raw) * 100 / (ADC_DRY_VALUE - ADC_WET_VALUE);
}

void app_main(void)
{
    // Initialize ADC for soil moisture sensor
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){.unit_id = ADC_UNIT_1}, &adc_handle);
    adc_oneshot_config_channel(adc_handle, MOISTURE_ADC_CHANNEL, &(adc_oneshot_chan_cfg_t){
        .atten    = MOISTURE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    });
    // Initialize ADC calibration
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = MOISTURE_ADC_CHANNEL,
        .atten    = MOISTURE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
    // Main loop: read ADC value, convert to voltage and percentage, print results
    while (1) {
        int raw = 0;
        adc_oneshot_read(adc_handle, MOISTURE_ADC_CHANNEL, &raw);

        int voltage_mv = 0;
        adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);

        printf("Raw: %4d | Voltage: %4d mV | Moisture: %3d%%\n",
               raw, voltage_mv, raw_to_percent(raw));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}