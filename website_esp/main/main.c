#include "big_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "web_server.h"
#include <stdio.h>

void app_main(void) {
    xTaskCreate(generate_big_data_task, "DataGen", 4096, NULL, 0, NULL);

    wifi_init_softap();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
