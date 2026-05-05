#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/spi_types.h"
#include "packet_def.h"
#include "sd_card.h"
#include "sd_protocol_types.h"
#include "sdmmc_cmd.h"
#include "time.h"
#include <dirent.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <time.h>

static const char *TAG = "SD_CARD";
static inline bool dir_exists(const char *path) { return (f_stat(path, NULL) == FR_OK); }
static inline void ensure_directory_exist(const char *year_path, const char *month_path) {
    if (!dir_exists(MOUNT_POINT DATA_DIR))
        mkdir(MOUNT_POINT DATA_DIR, 700);
    if (!dir_exists(year_path))
        mkdir(year_path, 700);
    if (!dir_exists(month_path))
        mkdir(month_path, 700);
}

static char *find_first_entry(const char *path, const char *format_pattern) {
    struct dirent **namelist;
    char *result = NULL;

    int n = scandir(path, &namelist, NULL, alphasort);
    if (n < 0)
        return NULL;

    for (int i = 0; i < n; i++) {
        if (result == NULL) {
            int dummy;
            if (sscanf(namelist[i]->d_name, format_pattern, &dummy) == 1) {
                result = strdup(namelist[i]->d_name);
            }
        }
        free(namelist[i]);
    }
    free(namelist);
    return result;
}

struct tm *oldest_log_tm = NULL;
bool find_oldest_log(struct tm *a) {
    if (oldest_log_tm != NULL) {
        memcpy(a, oldest_log_tm, sizeof(struct tm));
        return true;
    }
    char path[128];

    char *year_str = find_first_entry(MOUNT_POINT DATA_DIR, "%d");
    if (!year_str)
        return false;
    oldest_log_tm->tm_year = atoi(year_str) - 1900;
    snprintf(path, sizeof(path), MOUNT_POINT DATA_DIR "/%s", year_str);
    free(year_str);

    char *month_str = find_first_entry(path, "%d");
    if (!month_str)
        return false;
    oldest_log_tm->tm_mon = atoi(month_str) - 1;
    strlcat(path, "/", sizeof(path));
    strlcat(path, month_str, sizeof(path));
    free(month_str);

    char *file_str = find_first_entry(path, "%d-data.bin");
    if (!file_str)
        return false;
    oldest_log_tm->tm_mday = atoi(file_str);

    memcpy(a, oldest_log_tm, sizeof(struct tm));
    free(file_str);
    return true;
}

esp_err_t b_append_file(full_packet_time_t data) {
    // Get the current time
    struct timespec ts = {.tv_sec = data.time};
    struct tm timeinfo = {0};
    localtime_r(&ts.tv_sec, &timeinfo);

    char year_path[20] = {0};
    char month_path[25] = {0};
    char full_path[48] = {0};
    snprintf(year_path, 20, "%s/%04d", MOUNT_POINT DATA_DIR, (timeinfo.tm_year + 1900) & 0xFFF);
    snprintf(month_path, 25, "%s/%02d", year_path, (timeinfo.tm_mon + 1) & 0xF);
    snprintf(full_path, 48, "%s/%02d-data.bin", month_path, timeinfo.tm_mday & 0x2F);

    ensure_directory_exist(year_path, month_path);

    ESP_LOGD(TAG, "Opening file to append, '%s'", full_path);
    FILE *f = fopen(full_path, "ab");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file, '%s'", full_path);
        return ESP_FAIL;
    }

    fwrite(&data, sizeof(full_packet_time_t), 1, f);
    fclose(f);
    return ESP_OK;
}

READ_RETURN_STATE b_read_date(uint8_t *result, struct tm timeinfo, size_t start, size_t *len) {
    char path[48] = {0};
    snprintf(path, 48, "%s/%04d/%02d/%02d-data.bin", MOUNT_POINT DATA_DIR, (timeinfo.tm_year + 1900) & 0xFFF, (timeinfo.tm_mon + 1) & 0xF,
             timeinfo.tm_mday & 0x2F);
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Did not find file :  %s", path);
        return READ_FAILURE;
    }
    const size_t max_read = *len;
    const size_t read_amount = max_read / sizeof(full_packet_time_t);

    fseek(f, start, SEEK_SET);
    size_t segments_read = fread(result, sizeof(full_packet_time_t), read_amount, f);
    fclose(f);

    *len = segments_read * sizeof(full_packet_time_t);

    if (read_amount == segments_read)
        return READ_NOT_DONE;
    return READ_DONE;
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
    size_t packet_size = 4 + sizeof(full_packet_t); // 4 comes from time stamp
    READ_RETURN_STATE return_state;
    while (true) {
        if (cur_idx + packet_size > max_size) {
            return_state = READ_NOT_DONE;
            break;
        }
        size_t bytes_read = fread(&result[cur_idx], 1, packet_size, f);
        if (bytes_read == 0) {
            return_state = READ_DONE;
            break;
        }
        cur_idx += bytes_read;
    }
    fclose(f);
    *len = cur_idx;
    return return_state;
}

static sdmmc_card_t *card;
void init_sd_card() {
    ESP_LOGI(TAG, "Initializing SD Card");
    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

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
}
