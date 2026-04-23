#ifndef MOIST_SOIL
#define MOIST_SOIL

#include "esp_err.h"
#include "packet_def.h"

#define MOISTURE_ADC_ATTEN ADC_ATTEN_DB_12

esp_err_t get_moist_read(sensor_payload_t *payload);

#endif // !MOIST_SOIL
