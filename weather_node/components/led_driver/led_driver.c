/**
 * ws2812_led.c
 * Minimal single-LED WS2812 driver for ESP32-H2 DevKit (GPIO 8)
 *
 * Protocol (WS2812B datasheet):
 *   Bit-1  : 800 ns HIGH, 450 ns LOW  (T1H/T1L)
 *   Bit-0  : 400 ns HIGH, 850 ns LOW  (T0H/T0L)
 *   Reset  : LOW > 50 µs
 *   Byte order sent to the wire: G → R → B  (24 bits total)
 *
 * Clock:  10 MHz  →  1 tick = 100 ns
 *   T1H = 8 ticks  (800 ns)   T1L = 4 ticks (400 ns ... ~450 ns ok)
 *   T0H = 4 ticks  (400 ns)   T0L = 8 ticks (800 ns ... ~850 ns ok)
 *   Reset = 600 ticks (60 µs) — well above the 50 µs minimum
 */

#include "led_driver.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <string.h>

static const char *TAG = "ws2812";

/* ── Timing constants (ticks at 10 MHz = 100 ns each) ──────────── */
#define T1H 8      /* 800 ns */
#define T1L 4      /* 400 ns */
#define T0H 4      /* 400 ns */
#define T0L 8      /* 800 ns */
#define TRESET 600 /* 60 µs  */

/* ── RMT handles ────────────────────────────────────────────────── */
static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

/* ── Encoder internals ──────────────────────────────────────────── */

/*
 * We use a "bytes encoder" backed by two copy-encoders:
 *   • bytes_enc  – turns each byte into 8 RMT symbols (bit by bit)
 *   • reset_enc  – emits the >50 µs reset pulse after the data
 *
 * All of this lives in a tiny custom encoder struct so we stay
 * independent of led_strip or any other component.
 */

typedef struct {
    rmt_encoder_t base; /* MUST be first member */
    rmt_encoder_handle_t bytes_enc;
    rmt_encoder_handle_t reset_enc;
    int state; /* 0 = data, 1 = reset */
} ws2812_encoder_t;

/* The single reset symbol (60 µs LOW). */
static const rmt_symbol_word_t RESET_SYM = {
    .level0 = 0,
    .duration0 = TRESET,
    .level1 = 0,
    .duration1 = 1, /* RMT requires duration > 0 */
};

/* encode() callback — called by the RMT driver during transmission */
static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state) {
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes = enc->bytes_enc;
    rmt_encoder_handle_t reset = enc->reset_enc;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded = 0;

    if (enc->state == 0) {
        encoded += bytes->encode(bytes, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 1; /* move to reset phase */
            session_state &= ~RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = session_state;
            return encoded;
        }
    }

    if (enc->state == 1) {
        encoded += reset->encode(reset, channel, &RESET_SYM, sizeof(RESET_SYM), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = RMT_ENCODING_RESET; /* back to 0 */
            session_state |= RMT_ENCODING_COMPLETE;
        }
    }

    *ret_state = session_state;
    return encoded;
}

static esp_err_t ws2812_delete(rmt_encoder_t *encoder) {
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(enc->bytes_enc);
    rmt_del_encoder(enc->reset_enc);
    free(enc);
    return ESP_OK;
}

static esp_err_t ws2812_reset(rmt_encoder_t *encoder) {
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(enc->bytes_enc);
    rmt_encoder_reset(enc->reset_enc);
    enc->state = 0;
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────── */

esp_err_t ws2812_init(void) {
    if (s_rmt_chan) {
        ESP_LOGW(TAG, "already initialised");
        return ESP_OK;
    }

    /* 1. Create RMT TX channel */
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = WS2812_GPIO_NUM,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812_RMT_RES_HZ,
        .mem_block_symbols = 48, /* enough for 1 LED (24 bits) + reset */
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s_rmt_chan), TAG, "create TX channel failed");

    /* 2. Build the custom WS2812 encoder */
    ws2812_encoder_t *enc = calloc(1, sizeof(*enc));
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_NO_MEM, TAG, "no memory for encoder");

    enc->base.encode = ws2812_encode;
    enc->base.del = ws2812_delete;
    enc->base.reset = ws2812_reset;

    /* Bytes sub-encoder: bit1 and bit0 symbol definitions */
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit1 = {.level0 = 1, .duration0 = T1H, .level1 = 0, .duration1 = T1L},
        .bit0 = {.level0 = 1, .duration0 = T0H, .level1 = 0, .duration1 = T0L},
        .flags.msb_first = true, /* WS2812 sends MSB first */
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes_enc), TAG, "create bytes encoder failed");

    /* Copy sub-encoder used for the reset pulse */
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &enc->reset_enc), TAG, "create copy encoder failed");

    s_encoder = &enc->base;

    /* 3. Enable the channel */
    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_chan), TAG, "enable channel failed");

    ESP_LOGI(TAG, "WS2812 driver ready on GPIO%d", WS2812_GPIO_NUM);
    return ESP_OK;
}

esp_err_t ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (!s_rmt_chan || !s_encoder) {
        ESP_LOGE(TAG, "call ws2812_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    /* WS2812 wire order: Green → Red → Blue */
    uint8_t grb[3] = {green, red, blue};

    rmt_transmit_config_t tx = {
        .loop_count = 0, /* single shot */
    };

    ESP_RETURN_ON_ERROR(rmt_transmit(s_rmt_chan, s_encoder, grb, sizeof(grb), &tx), TAG, "transmit failed");

    /* Wait for the frame (including the reset) to finish before returning */
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_rmt_chan, portMAX_DELAY), TAG, "wait failed");

    return ESP_OK;
}

static TickType_t led_turn_off = 0;

esp_err_t ws2812_off(void) {
    if (led_turn_off < xTaskGetTickCount()) {
        return ws2812_set_color(0, 0, 0);
    }
    return ESP_OK;
}

#define LED_POST_PONE()                                                                                                                    \
    do {                                                                                                                                   \
        led_turn_off = xTaskGetTickCount() + pdMS_TO_TICKS(1000);                                                                          \
    } while (0);

esp_err_t ws2812_on_green(void) {
    LED_POST_PONE();
    return ws2812_set_color(0, 50, 0);
}
esp_err_t ws2812_on_red(void) {
    LED_POST_PONE();
    return ws2812_set_color(50, 0, 0);
}
esp_err_t ws2812_on_yellow(void) {
    LED_POST_PONE();
    ws2812_set_color(50, 15, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return ws2812_on_pink();
}
esp_err_t ws2812_on_blue(void) {
    LED_POST_PONE();
    return ws2812_set_color(0, 0, 50);
}
esp_err_t ws2812_on_pink(void) {
    LED_POST_PONE();
    return ws2812_set_color(50, 0, 50);
}

esp_err_t ws2812_deinit(void) {
    if (s_rmt_chan) {
        rmt_disable(s_rmt_chan);
        rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
    }
    if (s_encoder) {
        s_encoder->del(s_encoder);
        s_encoder = NULL;
    }
    return ESP_OK;
}
