#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define HALL_PIN 1

void app_main(void) {
    gpio_set_direction(HALL_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(HALL_PIN); // internal pull-up
    while (1) {
        int level = gpio_get_level(HALL_PIN);
        printf("Hall level: %d\n", level);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
