#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "pcnt_sensor.h"
#include "portmacro.h"
#include "soc/soc_caps.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static char *TAG = "PCNT";

static pcnt_unit_handle_t rain_pcnt_unit = NULL;
static bool rain_init = false;

static void rain_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &rain_pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = RAIN_PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(rain_pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_ERROR_CHECK(pcnt_unit_enable(rain_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(rain_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(rain_pcnt_unit));
    rain_init = true;
}

static int64_t rain_last_time = 0;
uint16_t get_rain() {
    if (!rain_init) {
        ESP_LOGW(TAG, "Rain is not initialized");
        rain_pcnt_init();
        return 0;
    }
    int rain_cnt = 0;
    int64_t cur_time = esp_timer_get_time();
    int64_t duration = cur_time - rain_last_time;

    ESP_ERROR_CHECK(pcnt_unit_get_count(rain_pcnt_unit, &rain_cnt));
    pcnt_unit_clear_count(rain_pcnt_unit);

    if (duration <= 0)
        return 0;

    double rain_rate = (((double)rain_cnt * 0.2) / (double)duration) * 3600000000.0;
    ESP_LOGI(TAG, "Rain='%.1f'mm   pulses='%d'", rain_rate, rain_cnt);
    rain_last_time = cur_time;
    return (uint16_t)(rain_rate * 10);
}

static pcnt_unit_handle_t wind_pcnt_unit = NULL;

/**
 * @brief The init function for the windPcnt counter
 * @note Is usually called before starting the windPcntTask
 **/
static void wind_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &wind_pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = WIND_PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(wind_pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_ERROR_CHECK(pcnt_unit_enable(wind_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(wind_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(wind_pcnt_unit));
}

uint16_t wind_shared_speed = 0;

void pcnt_task(void *duty_cycle_ms) {
    wind_pcnt_init();
    rain_pcnt_init();

    int wind_cnt = 0;
    int64_t cur_time = 0, last_time = 0;
    float mms = 0;
    TickType_t PreviousWakeTime = xTaskGetTickCount();

    while (1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(wind_pcnt_unit, &wind_cnt));
        cur_time = esp_timer_get_time();
        ESP_ERROR_CHECK(pcnt_unit_clear_count(wind_pcnt_unit));

        mms = PI * WIND_CUP_DIAMETER * CALIBRATION_FACTOR * wind_cnt * 1000000000ULL / (cur_time - last_time);
        wind_shared_speed = (uint16_t)(mms);
        ESP_LOGI(TAG, "pulses = %ld, mms = %f, shared = %d", wind_cnt, (double)mms, wind_shared_speed);

        last_time = esp_timer_get_time();
        vTaskDelayUntil(&PreviousWakeTime, pdMS_TO_TICKS((uint32_t *)duty_cycle_ms));
    }
    ESP_LOGW(TAG, "Left task unexpectedly");
}
