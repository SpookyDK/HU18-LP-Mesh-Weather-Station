#pragma once

#include "esp_err.h"
#include "i2cdev.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * BMP280 I2C addresses.
 * SDO pin low  -> 0x76
 * SDO pin high -> 0x77 (GY-91 default)
 */
#define BMP280_I2C_ADDR_SDO_LOW (0x76)
#define BMP280_I2C_ADDR_SDO_HIGH (0x77)

/** BMP280 chip ID returned by the WHO_AM_I register */
#define BMP280_CHIP_ID (0x58)
/** BME280 chip ID (same silicon, humidity added) */
#define BME280_CHIP_ID (0x60)

/**
 * @brief Oversampling settings.
 *
 * Applied to both temperature and pressure measurements.
 * Higher oversampling = lower noise, higher current draw, longer conversion.
 */
typedef enum {
    BMP280_SKIPPED = 0,         /**< Output set to 0x80000 */
    BMP280_ULTRA_LOW_POWER = 1, /**< x1  oversampling */
    BMP280_LOW_POWER = 2,       /**< x2  oversampling */
    BMP280_STANDARD = 3,        /**< x4  oversampling */
    BMP280_HIGH_RES = 4,        /**< x8  oversampling */
    BMP280_ULTRA_HIGH_RES = 5,  /**< x16 oversampling */
} bmp280_oversampling_t;

/**
 * @brief IIR filter coefficient.
 *
 * Filters out short-term disturbances (door slams, wind gusts).
 * Coefficient 0 = filter off.
 */
typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4,
} bmp280_filter_t;

/**
 * @brief Standby time between measurements in normal mode.
 */
typedef enum {
    BMP280_STANDBY_05 = 0,   /**< 0.5  ms */
    BMP280_STANDBY_62 = 1,   /**< 62.5 ms */
    BMP280_STANDBY_125 = 2,  /**< 125  ms */
    BMP280_STANDBY_250 = 3,  /**< 250  ms */
    BMP280_STANDBY_500 = 4,  /**< 500  ms */
    BMP280_STANDBY_1000 = 5, /**< 1000 ms */
    BMP280_STANDBY_2000 = 6, /**< 2000 ms */
    BMP280_STANDBY_4000 = 7, /**< 4000 ms */
} bmp280_standby_t;

/**
 * @brief Sensor operating mode.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,  /**< No measurements, lowest power */
    BMP280_MODE_FORCED = 1, /**< One measurement then back to sleep */
    BMP280_MODE_NORMAL = 3, /**< Continuous measurements at standby interval */
} bmp280_mode_t;

/**
 * @brief Configuration parameters for the BMP280.
 */
typedef struct {
    bmp280_mode_t mode;
    bmp280_filter_t filter;
    bmp280_oversampling_t oversampling_pressure;
    bmp280_oversampling_t oversampling_temperature;
    bmp280_standby_t standby;
} bmp280_config_t;

/**
 * @brief Factory trim calibration data read from the sensor at init.
 *
 * These are unique to each device and must be used for compensation.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_t;

/**
 * @brief BMP280 device descriptor.
 *
 * Embed one of these per sensor instance. Zero-init before calling
 * bmp280_init_desc(), then pass by pointer to all API functions.
 */
typedef struct {
    i2c_dev_t i2c_dev;    /**< i2cdev descriptor — must be first */
    bmp280_calib_t calib; /**< Trim calibration (populated at init) */
    int32_t t_fine;       /**< Intermediate temperature used by pressure compensation */
} bmp280_dev_t;

/**
 * @brief Default configuration: normal mode, x4 oversampling, filter off, 250 ms standby.
 */
#define BMP280_DEFAULT_CONFIG()                                                                                        \
    {                                                                                                                  \
        .mode = BMP280_MODE_NORMAL,                                                                                    \
        .filter = BMP280_FILTER_OFF,                                                                                   \
        .oversampling_pressure = BMP280_STANDARD,                                                                      \
        .oversampling_temperature = BMP280_STANDARD,                                                                   \
        .standby = BMP280_STANDBY_250,                                                                                 \
    }

/* ---- API ---------------------------------------------------------------- */

/**
 * @brief Initialise the i2cdev descriptor.
 *
 * Call this before bmp280_init().
 *
 * @param dev     Device handle (caller-allocated).
 * @param port    I2C port number (e.g. I2C_NUM_0).
 * @param addr    I2C address (BMP280_I2C_ADDR_SDO_LOW / HIGH).
 * @param sda_gpio SDA GPIO number.
 * @param scl_gpio SCL GPIO number.
 * @return ESP_OK on success.
 */
esp_err_t bmp280_init_desc(bmp280_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free resources held by the i2cdev descriptor.
 *
 * @param dev Device handle.
 * @return ESP_OK on success.
 */
esp_err_t bmp280_free_desc(bmp280_dev_t *dev);

/**
 * @brief Probe, read calibration, and apply configuration.
 *
 * Must be called after bmp280_init_desc() and i2cdev_init().
 *
 * @param dev    Device handle.
 * @param config Desired configuration. Pass NULL for BMP280_DEFAULT_CONFIG.
 * @return ESP_OK on success.
 */
esp_err_t bmp280_init(bmp280_dev_t *dev, const bmp280_config_t *config);

/**
 * @brief Apply a new configuration to a running device.
 *
 * The sensor is briefly put into sleep mode while registers are written.
 *
 * @param dev    Device handle (must be initialised).
 * @param config New configuration.
 * @return ESP_OK on success.
 */
esp_err_t bmp280_set_config(bmp280_dev_t *dev, const bmp280_config_t *config);

/**
 * @brief Trigger a single forced-mode measurement.
 *
 * Only meaningful when the device is configured with BMP280_MODE_FORCED.
 * Blocks until measurement is complete (~40 ms for x4 oversampling).
 *
 * @param dev Device handle.
 * @return ESP_OK on success.
 */
esp_err_t bmp280_force_measurement(bmp280_dev_t *dev);

/**
 * @brief Read compensated temperature and pressure.
 *
 * @param dev         Device handle.
 * @param temperature Degrees Celsius (may be NULL).
 * @param pressure    Pascals (may be NULL).
 * @return ESP_OK on success.
 */
esp_err_t bmp280_read(bmp280_dev_t *dev, int32_t *temperature, uint32_t *pressure);
