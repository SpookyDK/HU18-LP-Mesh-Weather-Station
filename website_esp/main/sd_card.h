#ifndef SD_CARD_MINE
#define SD_CARD_MINE

#include "esp_err.h"
#include "packet_def.h"

#define MOUNT_POINT "/sdcard"

#define PIN_MISO 5
#define PIN_MOSI 4
#define PIN_CLK 6
#define PIN_CS 10

esp_err_t b_append_file(const char *path, full_packet_t data[], uint8_t count);
void init_sd_card();

#endif // !SD_CARD_MINE
