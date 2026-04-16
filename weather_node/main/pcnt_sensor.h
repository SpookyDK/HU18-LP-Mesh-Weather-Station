#ifndef PCNT_SENSOR_H
#define PCNT_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT -1000
#define WIND_CUP_DIAMETER 0.97f
#define CALIBRATION_FACTOR 0.75f
#define PI 3.1415926f

uint16_t get_rain();
/**
 * @brief The task function to measure the wind,
 * @Important  The windPcntInit function needs to be called before starting the
 * task
 * @paramin uint32_t timedelay:  The time between measurements ms
 **/
void pcnt_task(void *duty_cycle_ms);

#endif // !PCNT_SENSOR_H
