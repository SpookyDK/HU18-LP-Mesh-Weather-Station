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
#include "tsl2591.h"
#include <stdint.h>

static bool is_i2c_init = false;

static inline void init_i2c(void) {
    if (is_i2c_init) {
        return;
    }
    i2cdev_init();
    is_i2c_init = true;
}

/**
 * This Var is set by the barometer_task.
 * This value is not tha actual value but the difference from 100 kPa.
 * Example 1: if the value is "1136" the proces to get the actual pressure is: 1136 + 100000 = 101136 Pa
 */
int16_t bmp_shared_pressure = 0;
bool bmp_shared_status = true;
// static const char *TAG = "Bar";

void barometer_task(void *duty_cycle_ms) {
    init_i2c();

    bmp280_dev_t bmp = {0};
    ESP_ERROR_CHECK(bmp280_init_desc(&bmp, I2C_MASTER_NUM, BMP280_I2C_ADDR_SDO_LOW, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    uint8_t attempts = 0;
    while (i2c_dev_check_present(&bmp.i2c_dev) != ESP_OK && ++attempts < 3) {
        if (attempts == 3) {
            ESP_LOGW("Bar", "Giving up on task, Terminating");
            bmp_shared_status = true;
            vTaskDelete(NULL);
        }
        ESP_LOGE("Bar", "Failed to find dev");
    }
    bmp_shared_status = false;

    bmp280_config_t bmp_cfg = {
        .mode = BMP280_MODE_NORMAL,
        .filter = BMP280_FILTER_4,
        .oversampling_pressure = BMP280_STANDARD,
        .oversampling_temperature = BMP280_STANDARD,
        .standby = BMP280_STANDBY_250,
    };
    ESP_ERROR_CHECK(bmp280_init(&bmp, &bmp_cfg));

    int32_t bmp_tempeture;
    uint32_t bmp_pressure;

    // This is a dummy reading to prime the actual reeadings
    bmp280_read(&bmp, &bmp_tempeture, &bmp_pressure);
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_err_t res;
    while (1) {
        res = bmp280_read(&bmp, &bmp_tempeture, &bmp_pressure);
        if (res == ESP_OK) {
            bmp_shared_pressure = (bmp_pressure >> 8) - 100000;
            ESP_LOGI("BMP280", "Read Values: temp = %2d °C, pres_diff = %d, Raw=%.2f", bmp_tempeture, bmp_shared_pressure,
                     (float)(bmp_pressure / 256.0f));
        } else {
            ESP_LOGE("BMP280", "Read Failed: %s", esp_err_to_name(res));
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms));
    }
    bmp280_free_desc(&bmp);
    ESP_LOGW("BMP280", "Leaving Accel Task");
}

uint16_t tsl_shared_spectrum = 0;

void light_sensor_task(void *duty_cycle_ms) {
    const char *TAG = "Light";
    // Ensure I2C is stated
    init_i2c();
    tsl2591_t light_sensor_dev = {0};
    light_sensor_dev.i2c_dev.cfg.master.clk_speed = 100000;

    ESP_ERROR_CHECK(tsl2591_init_desc(&light_sensor_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    if (i2c_dev_check_present(&light_sensor_dev.i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Could not find device");
        vTaskDelete(NULL);
    }
    ESP_ERROR_CHECK(tsl2591_init(&light_sensor_dev));
    ESP_ERROR_CHECK(tsl2591_set_power_status(&light_sensor_dev, TSL2591_POWER_ON));
    ESP_ERROR_CHECK(tsl2591_set_als_status(&light_sensor_dev, TSL2591_ALS_ON));

    ESP_ERROR_CHECK(tsl2591_set_gain(&light_sensor_dev, TSL2591_GAIN_LOW));
    ESP_ERROR_CHECK(tsl2591_set_integration_time(&light_sensor_dev, TSL2591_INTEGRATION_300MS));

    uint16_t cha0_full, cha1_ir;
    esp_err_t res;
    while (1) {
        if ((res = tsl2591_get_channel_data(&light_sensor_dev, &cha0_full, &cha1_ir)) == ESP_OK) {
            tsl_shared_spectrum = cha0_full;
            ESP_LOGI(TAG, "Full: %d shared_spectrum = %d  IR reading: %d", cha0_full, tsl_shared_spectrum, cha1_ir);
        } else {
            ESP_LOGW(TAG, "Cound not read Lux value: %d", res);
        }

        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms));
    }
}

static inline uint16_t i2c_swap16(uint16_t val) { return (val >> 8) | (val << 8); }
uint8_t power_sensor_addresses[4] = {0x40, 0x41, 0x44, 0x45};
struct power_sensor_value {
    uint16_t voltage;
    int16_t current;
    uint16_t power;
};

struct power_sensor_value power_sensor_values[4];

void power_sensor_task(void *duty_cycle_ms) {
    const char *TAG = "POWER";
    init_i2c();

    i2c_dev_t power_sensor_base = {
        .addr = 0x40,
        .port = I2C_MASTER_NUM,
        .cfg.scl_io_num = I2C_MASTER_SCL_IO,
        .cfg.sda_io_num = I2C_MASTER_SDA_IO,
        .cfg.master.clk_speed = 100 * 1000,
        .timeout_ticks = 2000,
    };

    esp_err_t ret;
    for (int i = 0; i < 4; i++) {
        power_sensor_base.addr = power_sensor_addresses[i];
        ret = i2c_dev_check_present(&power_sensor_base);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Try 1 No sensor at %02x", power_sensor_addresses[i]);
            ret = i2c_dev_check_present(&power_sensor_base);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Try 2 No sensor at %02x", power_sensor_addresses[i]);
                power_sensor_addresses[i] = 0;
                continue;
            }
        }
        ESP_LOGI("POWER_SENSOR", "Found a sensor at %02x", power_sensor_addresses[i]);
    }

    // write to the calibration register which is required or the device returns nothing.
    uint16_t cal = i2c_swap16(INA219_CALIBRATION_VALUE);
    for (int i = 0; i < 4; i++) {
        if (power_sensor_addresses[i] == 0) {
            continue;
        }
        power_sensor_base.addr = power_sensor_addresses[i];
        i2c_dev_write_reg(&power_sensor_base, INA219_REG_CALIBRATION, &cal, 2);
    }

    uint16_t raw_voltage = 0;
    uint16_t raw_current = 0;
    uint16_t raw_power = 0;
    while (1) {
        for (int i = 0; i < 4; i++) {
            // write to the calibration register which is required or the device returns nothing.
            if (power_sensor_addresses[i] == 0) {
                continue;
            }
            power_sensor_base.addr = power_sensor_addresses[i];

            i2c_dev_read_reg(&power_sensor_base, INA219_REG_BUSVOLTAGE, &raw_voltage, 2);
            i2c_dev_read_reg(&power_sensor_base, INA219_REG_CURRENT, &raw_current, 2);
            i2c_dev_read_reg(&power_sensor_base, INA219_REG_POWER, &raw_power, 2);

            // The first 2 bits are information flags, which are discarded
            power_sensor_values[i].voltage = (i2c_swap16(raw_voltage) >> 3) * 4;
            power_sensor_values[i].current = (int16_t)i2c_swap16(raw_current) * INA219_CURRENT_LSB_MA;
            power_sensor_values[i].power = i2c_swap16(raw_power) * INA219_POWER_LSB_MW;

            ESP_LOGE("POWER", "ADDR = %x Voltage: %d mV, Current: %d mA, Power: %d mW", power_sensor_addresses[i],
                     power_sensor_values[i].voltage, power_sensor_values[i].current, power_sensor_values[i].power);
        }
        vTaskDelay(pdMS_TO_TICKS((uint32_t *)duty_cycle_ms));
    }
    ESP_LOGE(TAG, "Task ended");
    vTaskDelete(NULL);
}
