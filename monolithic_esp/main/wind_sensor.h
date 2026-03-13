#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#define PCNT_INPUT_PIN 0

/**
 * @brief The init function for the windPcnt counter
 * @note Is usually called before starting the windPcntTask
 **/
static void wind_pcnt_init(void);

/**
 * @brief The task function to measure the wind,
 * @Important  The windPcntInit function needs to be called before starting the
 * task
 * @paramin uint32_t timedelay:  The time between measurements ms
 **/
void wind_task(void *duty_cycle_ms);

#endif // !WIND_SENSOR_H
