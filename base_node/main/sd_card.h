#ifndef SD_CARD_MINE
#define SD_CARD_MINE

#include "esp_err.h"
#include "packet_def.h"
#include <stdbool.h>
#include <stdint.h>

#define MOUNT_POINT "/sdcard"
#define DATA_DIR "/data"
#define PACKET_FILE MOUNT_POINT "/packets.bin"

#define PIN_MISO 5
#define PIN_MOSI 4
#define PIN_CLK 6
#define PIN_CS 10

typedef enum {
    READ_FAILURE = -1,
    READ_NOT_DONE,
    READ_DONE,

} READ_RETURN_STATE;

esp_err_t b_append_file(full_packet_time_t data);
READ_RETURN_STATE b_read_file(const char *path, size_t start_idx, size_t *len, uint8_t *result);
READ_RETURN_STATE b_read_date(uint8_t *result, struct tm timeinfo, size_t start, size_t *len);
esp_err_t b_read_last_packet(const char *path, full_packet_t *result);
void init_sd_card();

#endif // !SD_CARD_MINE
