#ifndef __DHT_H__
#define __DHT_H__

#include <driver/gpio.h>
#include <esp_err.h>

/**
 * @brief Read integer data from sensor on specified pin
 *
 * Humidity and temperature are returned as integers.
 * For example: humidity=625 is 62.5 %, temperature=244 is 24.4 degrees Celsius
 *
 * @param pin GPIO pin connected to sensor OUT
 * @param[out] humidity Humidity, percents * 10, nullable
 * @param[out] temperature Temperature, degrees Celsius * 10, nullable
 * @return `ESP_OK` on success
 */
esp_err_t dht_read_data(gpio_num_t pin, int16_t *humidity,
                        int16_t *temperature);

#endif // __DHT_H__
