#include "big_data.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "lora.h"
#include "packet_def.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static TaskHandle_t lora_task_handle = NULL;

static void IRAM_ATTR lora_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(lora_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void init_dio0_interrupt() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << CONFIG_DIO0_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_DIO0_GPIO, lora_isr_handler, NULL);
}

full_packet_t big_data_packet = {0};
static uint8_t buffer[256] = {0};

static void receive_task(void *p) {
    lora_init();
    lora_set_frequency(868e6);
    lora_set_spreading_factor(7);
    lora_set_bandwidth(125000);
    lora_set_coding_rate(5);
    lora_set_preamble_length(8);
    lora_set_sync_word(0x12);
    lora_set_tx_power(2);
    lora_enable_crc();
    lora_dump_registers();

    int packet_len;
    lora_receive();
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (lora_received()) {
            packet_len = lora_receive_packet(buffer, sizeof(buffer));
            if (packet_len > 0) {
                buffer[packet_len] = '\0';
                ESP_LOGI("RXtask", "len: '%d'", packet_len);
                if (packet_len == sizeof(full_packet_t)) {
                    memcpy(&big_data_packet, buffer, sizeof(full_packet_t));
                } else {
                    ESP_LOGW("RXtask", "Unkown packet");
                }
            }
            lora_receive();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW("RXtask", "Left task");
}

void generate_big_data_task() {
    init_dio0_interrupt();
    xTaskCreate(receive_task, "rx_task", 4096, NULL, 5, &lora_task_handle);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
