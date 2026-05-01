#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "packet_def.h"
#include "pcnt_sensor.h"
#include "pin_config.h"
#include "portmacro.h"
#include "soc/soc_caps.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static char *TAG = "PCNT";

static pcnt_unit_handle_t rain_pcnt_unit = NULL;
static bool rain_init = false;

static esp_err_t rain_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &rain_pcnt_unit), TAG, "RAIN> New unit");

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = RAIN_PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(rain_pcnt_unit, &chan_config, &pcnt_chan), TAG, "RAIN> New channel");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG,
                        "RAIN> Edge lording failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(rain_pcnt_unit), TAG, "RAIN> Enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(rain_pcnt_unit), TAG, "RAIN> Clearing");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(rain_pcnt_unit), TAG, "RAIN> Failed to start");
    rain_init = true;
    return ESP_OK;
}

static int64_t rain_last_time = 0;
esp_err_t get_rain_read(sensor_payload_t *payload) {
    if (!rain_init) {
        ESP_LOGW(TAG, "Rain is not initialized");
        if (rain_pcnt_init() != ESP_OK)
            return ESP_FAIL;
    }
    int rain_cnt = 0;
    int64_t cur_time = esp_timer_get_time();
    int64_t duration = cur_time - rain_last_time;

    if (duration <= 0)
        return ESP_FAIL;

    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(rain_pcnt_unit, &rain_cnt), TAG, "RAIN> Failed to read count");
    pcnt_unit_clear_count(rain_pcnt_unit);

    double rain_rate = (((double)rain_cnt * 0.2) / (double)duration) * 3600000000.0;
    // ESP_LOGI(TAG, "Rain='%.1f'mm   pulses='%d'", rain_rate, rain_cnt);
    rain_last_time = cur_time;
    payload->precipitation = (uint16_t)(rain_rate * 10);
    return ESP_OK;
}

static pcnt_unit_handle_t wind_pcnt_unit = NULL;
static bool initiated_wind = false;

/**
 * @brief The init function for the windPcnt counter
 * @note Is usually called before starting the windPcntTask
 **/
static esp_err_t wind_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &wind_pcnt_unit), TAG, "WIND> New unit");

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = WIND_PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(wind_pcnt_unit, &chan_config, &pcnt_chan), TAG, "WIND> New channel");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG,
                        "WIND> Edge lording failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(wind_pcnt_unit), TAG, "WIND> Enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(wind_pcnt_unit), TAG, "WIND> Clearing");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(wind_pcnt_unit), TAG, "WIND> Failed to start");
    initiated_wind = true;
    return ESP_OK;
}

static int64_t wind_last_time = 0;
esp_err_t get_wind_read(sensor_payload_t *payload) {
    if (!initiated_wind) {
        ESP_LOGW(TAG, "Wind is not initialized");
        if (wind_pcnt_init() != ESP_OK)
            return ESP_FAIL;
    }

    int wind_cnt = 0;
    int64_t cur_time = esp_timer_get_time();
    int64_t duration = cur_time - wind_last_time;
    double mms = 0;

    if (duration <= 0)
        return ESP_FAIL;

    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(wind_pcnt_unit, &wind_cnt), TAG, "WIND> Failed to read count");
    pcnt_unit_clear_count(wind_pcnt_unit);

    mms = PI * WIND_CUP_DIAMETER * CALIBRATION_FACTOR * wind_cnt * (double)(1000000000.0 / duration);
    payload->wind_speed = (uint16_t)(mms);
    // ESP_LOGI(TAG, "pulses = %ld, mms = %f, shared = %d", wind_cnt, (double)mms, wind_shared_speed);

    wind_last_time = cur_time;
    return ESP_OK;
}
