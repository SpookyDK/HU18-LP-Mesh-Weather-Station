#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"



#define Testpin 5

#define PCNT_INPUT_PIN 4 
#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT -1000

void app_main(void)
{
    gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << Testpin), 
    .mode = GPIO_MODE_OUTPUT,          
    .pull_up_en = GPIO_PULLUP_DISABLE, 
    .pull_down_en = GPIO_PULLDOWN_DISABLE,  
    .intr_type = GPIO_INTR_DISABLE             
    };
    gpio_config(&io_conf);


    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_INPUT_PIN,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));

    
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    int val = 0;
    while (1) {
        gpio_set_level(Testpin, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        gpio_set_level(Testpin, 0);

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &val));
        printf("Count: %d\n", val);

        vTaskDelay(pdMS_TO_TICKS(1000));

    }   
}