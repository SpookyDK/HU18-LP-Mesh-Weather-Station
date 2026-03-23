#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/spi_types.h"
#include "packet_def.h"
#include "sd_card.h"
#include "sd_protocol_types.h"
#include "sdmmc_cmd.h"
#include "time.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "SD_CARD";

#define MAX_BUF_SIZE 64

static esp_err_t s_write_file(const char *path, char *data) {
    ESP_LOGI(TAG, "Opening file for writing, '%s'", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file, '%s'", path);
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    return ESP_OK;
}

esp_err_t b_append_file(const char *path, full_packet_t data[], uint8_t count) {
    ESP_LOGI(TAG, "Opening file to append, '%s'", path);
    FILE *f = fopen(path, "ab");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file, '%s'", path);
        return ESP_FAIL;
    }
    // Get the current time
    struct timespec ts_now = {0};
    struct tm timeinfo = {0};
    clock_gettime(CLOCK_REALTIME, &ts_now);
    localtime_r(&ts_now.tv_sec, &timeinfo);

    fwrite((uint8_t *)&ts_now.tv_sec, sizeof(&ts_now.tv_sec), 1, f);
    fwrite((uint8_t *)&count, sizeof(uint8_t), 1, f);
    fwrite(data, sizeof(full_packet_t), count, f);
    fclose(f);
    return ESP_OK;
}

READ_RETURN_STATE b_read_file(const char *path, size_t start_idx, size_t *len, uint8_t *result) {
    ESP_LOGD(TAG, "Opening file for reading, '%s'", path);
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading, %s", path);
        return READ_FAILURE;
    }
    fseek(f, start_idx, SEEK_SET);
    size_t cur_idx = 0, max_size = *len;
    READ_RETURN_STATE return_state;
    while (true) {
        // Reading the time stamp, which is ignored
        size_t bytes_read = fread(&result[cur_idx], sizeof(uint8_t), 4, f);
        if (bytes_read == 0) {
            return_state = READ_DONE;
            *len = cur_idx;
            break;
        }
        cur_idx += bytes_read;
        // Reading the number of packets to read
        uint8_t count;
        fread(&count, sizeof(uint8_t), 1, f);
        result[cur_idx++] = count;
        // Ensuring there is enough space in buffer before reading
        if (cur_idx + sizeof(full_packet_t) * count >= max_size) {
            return_state = READ_NOT_DONE;
            break;
        }
        cur_idx += fread(&result[cur_idx], sizeof(uint8_t), sizeof(full_packet_t) * count, f);
        *len = cur_idx;
    }
    fclose(f);
    return return_state;
}

static sdmmc_card_t *card;
void init_sd_card() {
    ESP_LOGI(TAG, "Initializing SD Card");
    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    // host.max_freq_khz = SDMMC_FREQ_PROBING;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_CS;
    slot_cfg.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
    };

    ESP_LOGI(TAG, "Mounting file system");
    uint8_t attempts = 0;
    while (attempts++ < 5) {
        ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &card);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Filesystem mounted");
            break;
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to init the card (%s)", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    sdmmc_card_print_info(stdout, card);
    return;
    // The rest Is demonstration not indended for actual work

    const char *file_hello = MOUNT_POINT "/hello.txt";
    char data[MAX_BUF_SIZE];
    snprintf(data, MAX_BUF_SIZE, "Hello %s", card->cid.name);
    ret = s_write_file(file_hello, data);
    ESP_ERROR_CHECK(ret);

    char read_data[MAX_BUF_SIZE];
    // ret = s_read_file(file_hello, read_data);
    ESP_LOGI(TAG, "Retrieved string: %s", read_data);

    ESP_LOGI(TAG, "Unmounting Filesystem");
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card Unmounted");
}
