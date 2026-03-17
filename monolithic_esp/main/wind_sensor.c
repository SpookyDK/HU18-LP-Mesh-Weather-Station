#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "portmacro.h"
#include "wind_sensor.h"
#include <stdint.h>
#include <stdio.h>

#define WIND_CUP_DIAMETER 0.97f
#define CALIBRATION_FACTOR 0.75f
#define PI 3.1415926f
#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT -1000

static pcnt_unit_handle_t pcnt_unit = NULL;
static inline float calculate_wind_speed(uint32_t windrpm) {
    return (PI * WIND_CUP_DIAMETER * CALIBRATION_FACTOR * windrpm) / 60;
}

static void wind_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

uint16_t wind_shared_rpm = 0;
float wind_shared_ms = 0;

void wind_task(void *duty_cycle_ms) {
    wind_pcnt_init();

    int val = 0;
    int64_t lasttime = 0;
    int64_t time = 0;
    uint32_t rpm = 0;
    float ms = 0;
    ;

    while (1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &val)); // get count
        time = esp_timer_get_time(); // get high precision time
        rpm =
            (uint32_t)((val * 6000000ULL) / (time - lasttime)); // Calculate RPM
        ms = calculate_wind_speed(rpm);
        ESP_LOGI("Wind2",
                 "time1 = %lld, time2 = %lld, val = %ld, dif = %lld, rpm = "
                 "%ld, ms = %f",
                 lasttime, time, val, time - lasttime, rpm, ms);
        wind_shared_rpm = (uint16_t)rpm;
        wind_shared_ms = (uint16_t)ms;

        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));  // Reset the counter
        lasttime = esp_timer_get_time();                    // Note the time
        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms)); // wait
    }
}
