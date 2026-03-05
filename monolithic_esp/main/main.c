#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#define DUTY_CYCLES_MS 30000
#include "NEO_6M_UART.h"
#include "accel.h"
#include "moist_soil.h"
#include "tempeture.h"

static const char *TAG = "main";

void print_runtime_stats(void) {
    char buffer[1024];

    vTaskGetRunTimeStats(buffer);

    printf("Task Name\tRuntime\t\tCPU %%\n");
    printf("%s", buffer);
}

void app_main(void) {

    ESP_LOGI(TAG, "Starting Tempeture Task");
    xTaskCreate(temp_task, "tempTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Soil Moisture Reading Task");
    xTaskCreate(moist_task, "moistTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Accelerometer Task");
    xTaskCreate(accel_task, "accelTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    /* ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(gpsTask, "gpsTask", 4096, NULL, 0, NULL); */

    ESP_LOGI(TAG, "Finished starting all tasks");
    while (1) {
        // print_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(DUTY_CYCLES_MS));
    }
}
