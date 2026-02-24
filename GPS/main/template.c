#include "NEO_6M_UART.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include <driver/uart.h>
#include <stdint.h>
#include <stdio.h>
void print_runtime_stats(void) {
    char buffer[1024];

    vTaskGetRunTimeStats(buffer);

    printf("Task Name\tRuntime\tCPU %%\n");
    printf("%s\n", buffer);
}
void app_main(void) {
    // xTaskCreate(gpsInitUart, "initGPS", 4096, NULL, 0, NULL);
    gpsInitUart();
    xTaskCreate(gpsTask, "GPSTask", 2048, NULL, 0, NULL);
    // gpsTask();
    while (1) {
        print_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
