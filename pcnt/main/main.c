#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include <stdint.h>
#include <stdio.h>

#define PCNT_INPUT_PIN 5
#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT -1000

pcnt_unit_handle_t pcnt_unit = NULL;

/**
 * @brief The init function for the windPcnt counter
 * @note Is usually called before starting the windPcntTask
 **/
void windPcntInit(void) {
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
/**
 * @brief The task function to measure the wind,
 * @Important  The windPcntInit function needs to be called before starting the
 * task
 * @paramin uint32_t timedelay:  The time between measurements ms
 **/
void windPcntTask(uint32_t timedelay) {
    int val = 0;
    int64_t lasttime = 0;
    int64_t time = 0;
    uint32_t rpm = 0;
    uint32_t delay = pdMS_TO_TICKS(timedelay);
    while (1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &val)); // get count
        time = esp_timer_get_time();              // get high precision time
        rpm = val / (time - lasttime / 60000000); // Calculate RPM
        ESP_LOGI("Wind2",
                 "time1 = %lld, time2 = %lld, val = %ld, dif = %lld, rpm = %ld",
                 lasttime, time, val, time - lasttime, rpm);

        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); // Reset the counter
        lasttime = esp_timer_get_time();                   // Note the time
        vTaskDelay(pdMS_TO_TICKS(delay));                  // wait
    }
}
void app_main(void) {
    windPcntInit();
    windPcntTask(10000);
}
