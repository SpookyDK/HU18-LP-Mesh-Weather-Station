#include "big_data.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <stdint.h>

int16_t big_int_data = 0;
uint16_t big_uint_data = 0;

void generate_big_data_task() {
    while (1) {
        big_int_data = esp_random() % 2674 - 1337;
        big_uint_data = esp_random() % 1337;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
