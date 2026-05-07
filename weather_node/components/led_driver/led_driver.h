#ifndef LED_DRIVER_H
#define LED_DRIVER_H
/**
 * ws2812_led.h
 * Minimal single-LED WS2812 driver for ESP32-H2 DevKit (GPIO 8)
 *
 * Uses the ESP-IDF v5 RMT TX API directly — no led_strip component needed.
 * Only dependency: driver/rmt_tx.h and driver/rmt_encoder.h (both ship with
 * ESP-IDF; no extra idf_component.yml entry required).
 */

#include "esp_err.h"
#include <stdint.h>

/* ── Board config ───────────────────────────────────────────────── */
#define WS2812_GPIO_NUM 8            /* GPIO8 on ESP32-H2-DevKitM-1  */
#define WS2812_RMT_RES_HZ 10000000UL /* 10 MHz → 100 ns per tick */

/* ── Public API ─────────────────────────────────────────────────── */

/**
 * @brief Initialise the RMT channel and encoder for the onboard WS2812.
 *        Call once at startup before any ws2812_set_* calls.
 * @return ESP_OK on success.
 */
esp_err_t ws2812_init(void);

/**
 * @brief Send a GRB colour to the LED and latch it.
 *        Blocks until the RMT TX is complete (~30 µs for one LED).
 *
 * @param red   Red   channel 0-255
 * @param green Green channel 0-255
 * @param blue  Blue  channel 0-255
 * @return ESP_OK on success.
 */
esp_err_t ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue);

/** @brief Turn the LED off (convenience wrapper). */
esp_err_t ws2812_off(void);

/** @brief Turn the LED on — green at ~20 % brightness (easy on the eyes). */
esp_err_t ws2812_on_green(void);
esp_err_t ws2812_on_red(void);
esp_err_t ws2812_on_yellow(void);
esp_err_t ws2812_on_blue(void);

/**
 * @brief Release the RMT channel.  Call if you need to re-use the peripheral.
 */
esp_err_t ws2812_deinit(void);

#endif // !LED_DRIVER_H
