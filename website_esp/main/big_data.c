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
#include "pin_config.h"
#include "portmacro.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "RxTask";

static TaskHandle_t lora_task_handle = NULL;

// This should be set somewhere else, but prototype it fine
static uint8_t NETWORK_ID = 1;

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
        .pin_bit_mask = (1ULL << LORA_DIO0_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0_GPIO, lora_isr_handler, NULL);
}

#define CACHE_SIZE 32
#define CACHE_MASK (CACHE_SIZE - 1)

uint16_t packet_cache[CACHE_SIZE];
uint8_t write_index = 0;

static bool is_duplicate(uint16_t id) {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (packet_cache[i] == id) {
            return true;
        }
    }
    packet_cache[write_index] = id;
    write_index = (write_index + 1) & CACHE_MASK;
    return false;
}

full_packet_t big_data_packet = {0};
static uint8_t buffer[256] = {0};

static void receive_task(void *p) {
    init_dio0_interrupt();

    lora_init();
    lora_dump_registers();

    int packet_len;
    full_packet_t temp_packet;

    lora_receive();
    while (1) {
        lora_receive();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (lora_received()) {
            packet_len = lora_receive_packet(buffer, sizeof(buffer));
            if (packet_len != sizeof(full_packet_t)) {
                ESP_LOGW(TAG, "Unkown packet with len='%d'", packet_len);
                continue;
            }
            memcpy(&temp_packet, buffer, sizeof(full_packet_t));
            if (temp_packet.head.network_id != NETWORK_ID) {
                ESP_LOGI(TAG, "Ignoring packet from other network '%d'", temp_packet.head.network_id);
                continue;
            }
            if (is_duplicate(temp_packet.head.packet_id)) {
                ESP_LOGI(TAG, "Ignoring Duplicate packet id='%d'", temp_packet.head.packet_id);
                continue;
            }
            ESP_LOGI(TAG, "Received packet id='%d' from '%d'", temp_packet.head.packet_id, temp_packet.head.orig_node_id);
            memcpy(&big_data_packet, (uint8_t *)&temp_packet, packet_len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "Left task");
}

void start_receive_task() { xTaskCreate(receive_task, TAG, 4096, NULL, 5, &lora_task_handle); }
