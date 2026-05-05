#ifndef PCNT_SENSOR_H
#define PCNT_SENSOR_H

#include "esp_err.h"
#include "packet_def.h"
#include <stdint.h>

#define PCNT_HIGH_LIMIT 32767
#define PCNT_LOW_LIMIT -32767
#define WIND_CUP_DIAMETER 0.97
#define CALIBRATION_FACTOR 0.625474587957
#define PI 3.1415926

esp_err_t get_rain_read(sensor_payload_t *payload);
esp_err_t get_wind_read(sensor_payload_t *payload);

#endif // !PCNT_SENSOR_H
