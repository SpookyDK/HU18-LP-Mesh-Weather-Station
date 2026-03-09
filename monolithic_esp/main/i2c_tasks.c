#include "bmp280.h"
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

#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static bool is_i2c_init = false;

static void init_i2c(void) {
    if (is_i2c_init) {
        return;
    }
    i2cdev_init();
    is_i2c_init = true;
}

void accel_task(void *duty_cycle_ms) {
    init_i2c();

    bmp280_dev_t bmp = {0};
    ESP_ERROR_CHECK(
        bmp280_init_desc(&bmp, I2C_MASTER_NUM, BMP280_I2C_ADDR_SDO_LOW, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    bmp280_config_t bmp_cfg = {
        .mode = BMP280_MODE_NORMAL,
        .filter = BMP280_FILTER_4,
        .oversampling_pressure = BMP280_STANDARD,
        .oversampling_temperature = BMP280_STANDARD,
        .standby = BMP280_STANDBY_250,
    };
    ESP_ERROR_CHECK(bmp280_init(&bmp, &bmp_cfg));

    int32_t temp;
    uint32_t pres;

    // This is a dummy reading to prime the actual reeadings
    bmp280_read(&bmp, &temp, &pres);
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_err_t res;
    while (1) {
        res = bmp280_read(&bmp, &temp, &pres);
        if (res == ESP_OK) {
            ESP_LOGI("BMP280", "Read Values: temp = %2d °C, pres = %.2f Pa", temp, (float)(pres / 256.0f));
        } else {
            ESP_LOGE("BMP280", "Read Failed: %s", esp_err_to_name(res));
        }

        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms));
    }
    bmp280_free_desc(&bmp);
    ESP_LOGW("BMP280", "Leaving Accel Task");
}

static tsl2591_t light_sensor_dev;
static void init_light_sensor(void) {
    // Ensure I2C is stated
    init_i2c();
    light_sensor_dev.i2c_dev.cfg.master.clk_speed = 100000;

    ESP_ERROR_CHECK(tsl2591_init_desc(&light_sensor_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(tsl2591_init(&light_sensor_dev));

    ESP_ERROR_CHECK(tsl2591_set_power_status(&light_sensor_dev, TSL2591_POWER_ON));
    ESP_ERROR_CHECK(tsl2591_set_als_status(&light_sensor_dev, TSL2591_ALS_ON));

    ESP_ERROR_CHECK(tsl2591_set_gain(&light_sensor_dev, TSL2591_GAIN_MEDIUM));
    ESP_ERROR_CHECK(tsl2591_set_integration_time(&light_sensor_dev, TSL2591_INTEGRATION_300MS));
}

void light_sensor_task(void *duty_cycle_ms) {
    init_light_sensor();

    float lux; // Im so sorry, but this float will remain a float
    esp_err_t res;
    while (1) {
        if ((res = tsl2591_get_lux(&light_sensor_dev, &lux)) == ESP_OK) {
            ESP_LOGI("Light", "Lux reading: %f", lux);
        } else {
            ESP_LOGW("Light", "Cound not read Lux value: %d", res);
        }

        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms));
    }
}
