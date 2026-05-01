#ifndef PCNT_SENSOR_H
#define PCNT_SENSOR_H

#include "esp_err.h"
#include "packet_def.h"
#include <stdint.h>

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT -1000
#define WIND_CUP_DIAMETER 0.97f
#define CALIBRATION_FACTOR 0.75f
#define PI 3.1415926f

esp_err_t get_rain_read(sensor_payload_t *payload);
esp_err_t get_wind_read(sensor_payload_t *payload);

#endif // !PCNT_SENSOR_H
