#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "lora.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "SX1276";

void send_task(void *p) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        lora_send_packet((uint8_t *)"Hello", 5);
        ESP_LOGI(TAG, "Packet sent");
    }
}

uint8_t buffer[256] = {0};

void receive_task(void *p) {
    int x;
    lora_receive();
    lora_dump_registers();
    for (;;) {
        if (lora_received()) {
            x = lora_receive_packet(buffer, sizeof(buffer));
            if (x > 0) {
                buffer[x] = '\0';
                ESP_LOGI(TAG, "len: '%d'  Recieved: '%s'", x, buffer);
            }
            lora_receive();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    int ret = lora_init();
    ESP_LOGI(TAG, "Init returned %d", ret);

    lora_set_frequency(868e6);
    lora_set_spreading_factor(7);
    lora_set_bandwidth(125000);
    lora_set_coding_rate(5);
    lora_set_preamble_length(8);
    lora_set_sync_word(0x12);
    lora_set_tx_power(2);
    lora_enable_crc();

    lora_dump_registers();

    if (0) {
        xTaskCreate(send_task, "send_task", 2048, NULL, 5, NULL);
    } else {
        xTaskCreate(receive_task, "receive_task", 2048, NULL, 5, NULL);
    }
}
