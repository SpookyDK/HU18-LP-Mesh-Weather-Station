#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/spi_types.h"
#include "sd_card.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "SD_CARD";

#define MAX_BUF_SIZE 64

static esp_err_t s_write_file(const char *path, char *data) {
    ESP_LOGI(TAG, "Opening file for writing, '%s'", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file, %s", path);
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    return ESP_OK;
}

static esp_err_t s_read_file(const char *path, char *result) {
    ESP_LOGI(TAG, "Opening file for reading, '%s'", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading, %s", path);
        return ESP_FAIL;
    }
    char line[MAX_BUF_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);
    memcpy(result, line, strlen(line));
    return ESP_OK;
}

static sdmmc_card_t *card;
void init_sd_card() {
    // The sd card needs some power before starting
    vTaskDelay(pdMS_TO_TICKS(2000));

    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ESP_LOGI(TAG, "Initializing SD Card");
    ESP_LOGI(TAG, "Using SPI");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_device_handle_t dev_handle;
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 9000000, .mode = 0, .spics_io_num = PIN_CS, .queue_size = 1, .flags = 0, .pre_cb = NULL};

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Device to SPI bus");
        return;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_CS;
    slot_cfg.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting file system");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &card);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Filesystem mounted");
    } else if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to mount filesystem");
        return;
    } else {
        ESP_LOGE(TAG, "Failed to init the card (%s)", esp_err_to_name(ret));
        return;
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
    ret = s_read_file(file_hello, read_data);
    ESP_LOGI(TAG, "Retrieved string: %s", read_data);

    ESP_LOGI(TAG, "Unmounting Filesystem");
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card Unmounted");
}
