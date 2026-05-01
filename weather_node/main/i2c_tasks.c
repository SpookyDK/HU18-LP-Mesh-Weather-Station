#include "bmp280.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "i2c_tasks.h"
#include "i2cdev.h"
#include "my_drivers/bmp280.h"
#include "packet_def.h"
#include "pin_config.h"
#include "tsl2591.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static bool is_i2c_init = false;
const char *TAG = "I2CTasks";

#define ERROR_CHECK(func, msg)                                                                                                             \
    do {                                                                                                                                   \
        esp_err_t err_rc_ = (func);                                                                                                        \
        if (unlikely(err_rc_ != ESP_OK)) {                                                                                                 \
            ESP_LOGW(TAG, msg);                                                                                                            \
            return ESP_FAIL;                                                                                                               \
        }                                                                                                                                  \
    } while (0)

static inline void init_i2c(void) {
    if (is_i2c_init) {
        return;
    }
    i2cdev_init();
    is_i2c_init = true;
}

static bmp280_dev_t bmp = {0};
static bool initiated_bmp = false;

static esp_err_t init_barometer() {
    init_i2c();

    if (bmp280_init_desc(&bmp, I2C_MASTER_NUM, BMP280_I2C_ADDR_SDO_LOW, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to describe BMP");
        return ESP_FAIL;
    }
    uint8_t attempts = 0;
    while (i2c_dev_check_present(&bmp.i2c_dev) != ESP_OK && ++attempts <= 3) {
        if (attempts >= 3) {
            ESP_LOGW("Bar", "Failed to find Barometer, Giving up");
            return ESP_FAIL;
        }
        ESP_LOGW("Bar", "Failed to find bmp");
    }

    bmp280_config_t bmp_cfg = {
        .mode = BMP280_MODE_FORCED, // Hope this works, this should mean it will go to sleep after a reading
        .filter = BMP280_FILTER_4,
        .oversampling_pressure = BMP280_STANDARD,
        .oversampling_temperature = BMP280_STANDARD,
        .standby = BMP280_STANDBY_250,
    };
    if (bmp280_init(&bmp, &bmp_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start BMP280");
        return ESP_FAIL;
    }

    int32_t bmp_tempeture;
    uint32_t bmp_pressure;

    // This is a dummy reading to prime the actual reeadings
    bmp280_read(&bmp, &bmp_tempeture, &bmp_pressure);

    initiated_bmp = true;
    return ESP_OK;
}

esp_err_t get_pres_read(sensor_payload_t *payload) {
    if (!initiated_bmp) {
        ESP_LOGW(TAG, "BMP not initiated");
        if (init_barometer() != ESP_OK) {
            bmp280_free_desc(&bmp);
            return ESP_FAIL;
        }
    }

    int32_t bmp_tempeture;
    uint32_t bmp_pressure;

    if (bmp280_read(&bmp, &bmp_tempeture, &bmp_pressure) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get BMP reading");
        return ESP_FAIL;
    }
    payload->pressure = (int16_t)((bmp_pressure >> 8) - 100000);

    return ESP_OK;
}

static bool initiated_tsl = false;
static tsl2591_t light_sensor_dev = {0};

static esp_err_t init_tsl() {
    init_i2c();
    light_sensor_dev.i2c_dev.cfg.master.clk_speed = 100000;
    ERROR_CHECK(tsl2591_init_desc(&light_sensor_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO), "TSL> Init desc Failed");
    ERROR_CHECK(i2c_dev_check_present(&light_sensor_dev.i2c_dev), "TSL> device not found");
    ERROR_CHECK(tsl2591_init(&light_sensor_dev), "TSL> failed to init");
    ERROR_CHECK(tsl2591_set_power_status(&light_sensor_dev, TSL2591_POWER_ON), "TSL> Failed to set power state");
    ERROR_CHECK(tsl2591_set_als_status(&light_sensor_dev, TSL2591_ALS_ON), "TSL> Failed to set als");
    ERROR_CHECK(tsl2591_set_gain(&light_sensor_dev, TSL2591_GAIN_LOW), "TSL> Failed to set gain");
    ERROR_CHECK(tsl2591_set_integration_time(&light_sensor_dev, TSL2591_INTEGRATION_100MS), "TSL> Failed to set integration time");
    initiated_tsl = true;
    return ESP_OK;
}

esp_err_t get_light_read(sensor_payload_t *payload) {
    if (!initiated_tsl) {
        ESP_LOGW(TAG, "TSL> Not initiated");
        if (init_tsl() != ESP_OK) {
            ESP_LOGW(TAG, "TSL> Failed to init module, giving up");
            tsl2591_free_desc(&light_sensor_dev);
            return ESP_FAIL;
        }
    }
    uint16_t cha1_ir;
    float lux;
    ERROR_CHECK(i2c_dev_check_present(&light_sensor_dev.i2c_dev), "TSL> Missed light sensor in reading");
    ERROR_CHECK(tsl2591_get_lux(&light_sensor_dev, &lux, &cha1_ir), "TSL> Failed to convert reading");
    if (lux <= 1.0f) {
        payload->spectrum = 1;
    } else if (lux > 65535.0f) {
        payload->spectrum = 0xffff;
    } else {
        payload->spectrum = (uint16_t)lux;
    }
    // ESP_LOGI(TAG, "shared_spectrum = %d  IR reading: %d", tsl_shared_spectrum, cha1_ir);
    return ESP_OK;
}

static inline uint16_t i2c_swap16(uint16_t val) { return (val >> 8) | (val << 8); }
#define DEVICE_COUNT 2
static const uint8_t ADDRESSES[] = {POWER_ADDR_BATTERY, POWER_ADDR_SOLAR};
static i2c_dev_t devices[DEVICE_COUNT];
static bool devices_state[DEVICE_COUNT];
static bool initiated_power = false;

static esp_err_t init_power() {
    init_i2c();
    for (int i = 0; i < DEVICE_COUNT; i++) {
        devices[i].addr = ADDRESSES[i];
        devices[i].port = I2C_MASTER_NUM;
        devices[i].cfg.scl_io_num = I2C_MASTER_SCL_IO;
        devices[i].cfg.sda_io_num = I2C_MASTER_SDA_IO;
        devices[i].cfg.master.clk_speed = 100 * 1000;
        devices[i].timeout_ticks = 2000;
        if (i2c_dev_check_present(&devices[i]) == ESP_OK) {
            ESP_LOGI(TAG, "Found Power Sensor device addr='%02x'", devices[i].addr);
            devices_state[i] = true;
        } else {
            ESP_LOGW(TAG, "Failed to find Power Sensor device addr='%02x'", devices[i].addr);
            devices_state[i] = false;
        }
    }
    if (!(devices_state[0] | devices_state[1])) {
        ESP_LOGW(TAG, "POWER> Failed to init both devices, Giving up on power");
        return ESP_FAIL;
    }
    // write to the calibration register which is required or the device returns nothing.
    uint16_t cal = i2c_swap16(INA219_CALIBRATION_VALUE);
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (!devices_state[i])
            continue;
        i2c_dev_write_reg(&devices[i], INA219_REG_CALIBRATION, &cal, 2);
    }
    initiated_power = true;
    return ESP_OK;
}

esp_err_t get_power_read(sensor_payload_t *payload, uint8_t addr) {
    if (!initiated_power) {
        ESP_LOGW(TAG, "POWER> Not initiated");
        if (init_power() != ESP_OK)
            return ESP_FAIL;
    }
    i2c_dev_t *dev_ptr = NULL;
    // Find the device
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (devices[i].addr == addr && devices_state[i])
            dev_ptr = &devices[i];
    }
    if (dev_ptr == NULL) {
        ESP_LOGW(TAG, "POWER> Device not available");
        return ESP_FAIL;
    }

    uint16_t raw_voltage = 0, raw_power = 0, voltage = 0, power = 0;
    // uint16_t raw_current = 0;
    // int16_t current = 0;

    ERROR_CHECK(i2c_dev_read_reg(dev_ptr, INA219_REG_BUSVOLTAGE, &raw_voltage, 2), "POWER> Failed to read voltage");
    // ERROR_CHECK(i2c_dev_read_reg(dev_ptr, INA219_REG_CURRENT, &raw_current, 2), "POWER> Failed to read current");
    ERROR_CHECK(i2c_dev_read_reg(dev_ptr, INA219_REG_POWER, &raw_power, 2), "POWER> Failed to read power");

    // The first 2 bits of voltage is information flags, which are discarded
    voltage = (i2c_swap16(raw_voltage) >> 3) * INA219_VOLTAGE_LSB_MV;
    // current = (int16_t)(i2c_swap16(raw_current) * INA219_CURRENT_LSB_MA);
    power = i2c_swap16(raw_power) * INA219_POWER_LSB_MW;

    if (addr == POWER_ADDR_BATTERY) {
        payload->bat_voltage = voltage < 3500 ? 0 : (uint8_t)((voltage - 3500) >> 2);
    } else if (addr == POWER_ADDR_SOLAR) {
        payload->solar_output = (uint8_t)(power >> 2);
    }
    return ESP_OK;
}
