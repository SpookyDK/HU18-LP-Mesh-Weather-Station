#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "web_server.h"
#include <stdio.h>

void app_main(void) {
    connect_to_hotspot();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
