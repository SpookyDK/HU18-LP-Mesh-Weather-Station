#include "big_data.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
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
#include "sd_card.h"
#include "site_content.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_timeval.h>
#include <sys/time.h>
#include <time.h>

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

#define CACHE_SIZE 16
#define CACHE_MASK (CACHE_SIZE - 1)

full_packet_t packet_cache[CACHE_SIZE];
uint8_t cache_len = 0;

typedef enum {
    CACHE_NEW_ITEM,
    CACHE_DUPLICATE_PACKET,
    CACHE_RECURRING_NODE,
} cache_ret_state;

static cache_ret_state is_duplicate(full_packet_t pack) {
    for (int i = 0; i < cache_len; i++) {
        if (packet_cache[i].head.orig_node_id == pack.head.orig_node_id) {
            if (packet_cache[i].head.packet_id == pack.head.packet_id) {
                return CACHE_DUPLICATE_PACKET;
            }
            return CACHE_RECURRING_NODE;
        }
    }
    memcpy(&packet_cache[cache_len], &pack, sizeof(full_packet_t));
    cache_len = (cache_len + 1) & CACHE_MASK;
    return CACHE_NEW_ITEM;
}

full_packet_t big_data_packet = {0};
static uint8_t buffer[256] = {0};

static void receive_task(void *p) {
    init_dio0_interrupt();

    lora_init();
    lora_dump_registers();

    int packet_len;
    full_packet_t temp_packet;
    struct timespec ts_now = {0};
    struct tm timeinfo = {0};
    char strtime_buf[64] = {0};

    while (1) {
        lora_receive();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!lora_received()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
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
        cache_ret_state state = is_duplicate(temp_packet);
        if (state == CACHE_DUPLICATE_PACKET) {
            ESP_LOGI(TAG, "Ignoring Duplicate packet id='%d'", temp_packet.head.packet_id);
            continue;
        }

        clock_gettime(CLOCK_REALTIME, &ts_now);
        localtime_r(&ts_now.tv_sec, &timeinfo);
        strftime(strtime_buf, sizeof(strtime_buf), "%c", &timeinfo);

        ESP_LOGI(TAG, "Received packet id='%d' from '%d' at %s", temp_packet.head.packet_id, temp_packet.head.orig_node_id, strtime_buf);
        memcpy(&big_data_packet, (uint8_t *)&temp_packet, packet_len);
        if (state == CACHE_RECURRING_NODE) {
            if (b_append_file(PACKET_FILE, packet_cache, cache_len) == ESP_OK) {
                ESP_LOGI(TAG, "Stored buffer to SD card item_count: '%d'", cache_len);
                memcpy(&packet_cache[0], &temp_packet, sizeof(full_packet_t));
                cache_len = 1;
                // Notify site_content.c that it needs to send new data through websocket
                notify_ws_new_sd_data();
            } else {
                ESP_LOGW(TAG, "Failed to store packet cache");
            }
        }
    }
    ESP_LOGW(TAG, "Left task");
}

void start_receive_task() { xTaskCreate(receive_task, TAG, 4096, NULL, 5, &lora_task_handle); }

void read_last_packet() {
    // Read last packet to compare to
    esp_err_t ret;
    if ((ret = b_read_last_packet(PACKET_FILE, &big_data_packet)) == ESP_OK) {
        is_duplicate(big_data_packet);
        ESP_LOGI(TAG, "Read last packet, node_id='%d'  packet_id='%d'", big_data_packet.head.orig_node_id, big_data_packet.head.packet_id);
    }
}
