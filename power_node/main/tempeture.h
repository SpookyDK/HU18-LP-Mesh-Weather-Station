#ifndef TEMPETURE
#define TEMPETURE
#include "esp_err.h"
#include "packet_def.h"
#include <stdint.h>

esp_err_t get_air_reads(sensor_payload_t *packet);
esp_err_t get_soil_temp(sensor_payload_t *packet);

#define CONFIG_CONNECTION_TIMEOUT 5

#endif // !TEMPETURE
