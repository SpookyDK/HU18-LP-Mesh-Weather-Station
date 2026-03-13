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
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>

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

    int x;
    lora_receive();
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (lora_received()) {
            x = lora_receive_packet(buffer, sizeof(buffer));
            if (x > 0) {
                buffer[x] = '\0';
                ESP_LOGI("RXtask", "len: '%d'  Recieved: '%s'", x, buffer);
            }
            lora_receive();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW("RXtask", "Left task");
}

uint8_t big_data_bytearray[] = {0x0a, 0xc2, 0xf3, 0x05, 0x19, 0x9b, 0xfb, 0x21, 0x2c, 0xec, 0x00, 0x74, 0x01, 0x77,
                                0x01, 0x74, 0x01, 0x71, 0x01, 0x00, 0xc6, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00};

void generate_big_data_task() {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
