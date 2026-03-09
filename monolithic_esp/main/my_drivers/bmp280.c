#include "bmp280.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "bmp280";

/* ---- Register map ------------------------------------------------------- */
#define REG_CALIB_START (0x88) /**< First calibration byte (T1 LSB) */
#define REG_CHIP_ID (0xD0)
#define REG_RESET (0xE0)
#define REG_STATUS (0xF3)
#define REG_CTRL_MEAS (0xF4)
#define REG_CONFIG (0xF5)
#define REG_PRESS_MSB (0xF7) /**< First data byte; 6 bytes follow P+T */

#define RESET_WORD (0xB6)
#define CALIB_SIZE (24) /**< 12 × uint16_t trim words */

/* ---- Helpers ------------------------------------------------------------ */

/**
 * Write a single byte to a register.
 */
static inline esp_err_t write_reg(bmp280_dev_t *dev, uint8_t reg, uint8_t val) {
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
}

/**
 * Read one or more bytes starting at reg.
 */
static inline esp_err_t read_reg(bmp280_dev_t *dev, uint8_t reg, void *buf, size_t len) {
    return i2c_dev_read_reg(&dev->i2c_dev, reg, buf, len);
}

/* ---- Calibration -------------------------------------------------------- */

static esp_err_t read_calibration(bmp280_dev_t *dev) {
    uint8_t raw[CALIB_SIZE];
    esp_err_t ret = read_reg(dev, REG_CALIB_START, raw, CALIB_SIZE);
    if (ret != ESP_OK)
        return ret;

    bmp280_calib_t *c = &dev->calib;

    /* All calibration words are little-endian in the register map. */
    c->dig_T1 = (uint16_t)(raw[1] << 8) | raw[0];
    c->dig_T2 = (int16_t)((raw[3] << 8) | raw[2]);
    c->dig_T3 = (int16_t)((raw[5] << 8) | raw[4]);
    c->dig_P1 = (uint16_t)(raw[7] << 8) | raw[6];
    c->dig_P2 = (int16_t)((raw[9] << 8) | raw[8]);
    c->dig_P3 = (int16_t)((raw[11] << 8) | raw[10]);
    c->dig_P4 = (int16_t)((raw[13] << 8) | raw[12]);
    c->dig_P5 = (int16_t)((raw[15] << 8) | raw[14]);
    c->dig_P6 = (int16_t)((raw[17] << 8) | raw[16]);
    c->dig_P7 = (int16_t)((raw[19] << 8) | raw[18]);
    c->dig_P8 = (int16_t)((raw[21] << 8) | raw[20]);
    c->dig_P9 = (int16_t)((raw[23] << 8) | raw[22]);

    ESP_LOGD(TAG, "Calibration: T1=%u T2=%d T3=%d | P1=%u P2=%d", c->dig_T1, c->dig_T2, c->dig_T3, c->dig_P1,
             c->dig_P2);

    return ESP_OK;
}

/* ---- Bosch integer compensation (from BMP280 datasheet section 4.2.3) --- */

/**
 * Returns temperature in 0.01 °C increments.
 * Also populates dev->t_fine which is required by pressure compensation.
 */
static int32_t compensate_temperature(bmp280_dev_t *dev, int32_t adc_T) {
    const bmp280_calib_t *c = &dev->calib;
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) * ((int32_t)c->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)c->dig_T1) * ((adc_T >> 4) - (int32_t)c->dig_T1)) >> 12) * (int32_t)c->dig_T3) >>
           14;

    dev->t_fine = var1 + var2;
    return (dev->t_fine * 5 + 128) >> 8; /* °C × 100 */
}

/**
 * Returns pressure in Pa × 256.
 * Must be called after compensate_temperature() so t_fine is valid.
 */
static uint32_t compensate_pressure(const bmp280_dev_t *dev, int32_t adc_P) {
    const bmp280_calib_t *c = &dev->calib;
    int64_t var1, var2, p;

    var1 = (int64_t)dev->t_fine - 128000;
    var2 = var1 * var1 * (int64_t)c->dig_P6;
    var2 = var2 + ((var1 * (int64_t)c->dig_P5) << 17);
    var2 = var2 + (((int64_t)c->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->dig_P3) >> 8) + ((var1 * (int64_t)c->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)c->dig_P1) >> 33;

    if (var1 == 0)
        return 0; /* Avoid division by zero */

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)c->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)c->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)c->dig_P7 << 4);

    return (uint32_t)p; /* Pa × 256 */
}

/* ---- Public API --------------------------------------------------------- */

esp_err_t bmp280_init_desc(bmp280_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
    dev->i2c_dev.cfg.master.clk_speed = 400000; /* 400 kHz — BMP280 max */

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t bmp280_free_desc(bmp280_dev_t *dev) {
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmp280_init(bmp280_dev_t *dev, const bmp280_config_t *config) {
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    /* --- Verify chip identity ------------------------------------------- */
    uint8_t chip_id = 0;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg(dev, REG_CHIP_ID, &chip_id, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID 0x%02X (expected 0x%02X or 0x%02X)", chip_id, BMP280_CHIP_ID, BME280_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Found %s (ID=0x%02X)", chip_id == BME280_CHIP_ID ? "BME280" : "BMP280", chip_id);

    /* --- Soft reset -------------------------------------------------------- */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg(dev, REG_RESET, RESET_WORD));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    /* Datasheet: startup time after reset ≤ 2 ms */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* --- Read factory calibration ---------------------------------------- */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_calibration(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    /* --- Apply configuration ---------------------------------------------- */
    static const bmp280_config_t default_cfg = BMP280_DEFAULT_CONFIG();
    return bmp280_set_config(dev, config ? config : &default_cfg);
}

esp_err_t bmp280_set_config(bmp280_dev_t *dev, const bmp280_config_t *config) {
    if (!dev || !config)
        return ESP_ERR_INVALID_ARG;

    /*
     * ctrl_meas register (0xF4):
     *   [7:5] osrs_t  (temperature oversampling)
     *   [4:2] osrs_p  (pressure oversampling)
     *   [1:0] mode
     *
     * config register (0xF5):
     *   [7:5] t_sb    (standby time)
     *   [4:2] filter
     *   [0]   spi3w_en (leave 0)
     *
     * Registers must be written while sensor is in sleep mode.
     */
    uint8_t ctrl_meas = ((config->oversampling_temperature & 0x07) << 5) |
                        ((config->oversampling_pressure & 0x07) << 2) | (BMP280_MODE_SLEEP & 0x03);

    uint8_t config_reg = ((config->standby & 0x07) << 5) | ((config->filter & 0x07) << 2);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    /* Write sleep mode first so config register is writable */
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg(dev, REG_CTRL_MEAS, ctrl_meas));
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg(dev, REG_CONFIG, config_reg));

    /* Now set the actual mode (forced or normal) */
    if (config->mode != BMP280_MODE_SLEEP) {
        ctrl_meas = (ctrl_meas & ~0x03) | (config->mode & 0x03);
        I2C_DEV_CHECK(&dev->i2c_dev, write_reg(dev, REG_CTRL_MEAS, ctrl_meas));
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "ctrl_meas=0x%02X config=0x%02X", ctrl_meas, config_reg);
    return ESP_OK;
}

esp_err_t bmp280_force_measurement(bmp280_dev_t *dev) {
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    /* Read current ctrl_meas, then set mode bits to FORCED */
    uint8_t ctrl = 0;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg(dev, REG_CTRL_MEAS, &ctrl, 1));
    ctrl = (ctrl & ~0x03) | BMP280_MODE_FORCED;
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg(dev, REG_CTRL_MEAS, ctrl));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    /*
     * Wait for measurement to complete: poll the status register.
     * bit 3 (measuring) goes high during conversion and returns low when done.
     * Worst-case x16 oversampling for both channels is ~113 ms.
     */
    uint8_t status = 0;
    const TickType_t timeout = pdMS_TO_TICKS(200);
    const TickType_t deadline = xTaskGetTickCount() + timeout;

    do {
        vTaskDelay(pdMS_TO_TICKS(5));
        I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg(dev, REG_STATUS, &status, 1));
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

        if (xTaskGetTickCount() > deadline) {
            ESP_LOGE(TAG, "Forced measurement timed out");
            return ESP_ERR_TIMEOUT;
        }
    } while (status & (1 << 3)); /* measuring bit */

    return ESP_OK;
}

esp_err_t bmp280_read(bmp280_dev_t *dev, int32_t *temperature, uint32_t *pressure) {
    if (!dev || !temperature || !pressure)
        return ESP_ERR_INVALID_ARG;

    /*
     * Burst read: 6 bytes starting at 0xF7
     *   [0] press_msb  [1] press_lsb  [2] press_xlsb
     *   [3] temp_msb   [4] temp_lsb   [5] temp_xlsb
     * Raw values are 20-bit, left-aligned, stored in bits [19:4].
     */
    uint8_t raw[6];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg(dev, REG_PRESS_MSB, raw, sizeof(raw)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);

    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

    /* Temperature must always be compensated first — sets t_fine. */
    *temperature = compensate_temperature(dev, adc_T);
    *pressure = compensate_pressure(dev, adc_P);

    return ESP_OK;
}
