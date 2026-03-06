#include "bmx280.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "i2c_tasks.h"
#include "tsl2591.h"
#include <stdint.h>

#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "accel";

static bmx280_t *sensor;

static bool is_i2c_init = false;

static void init_i2c(void) {
    if (is_i2c_init) {
        return;
    }

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));
    is_i2c_init = true;
}

static void init_bmp280(void) {
    // Ensure I2C is started
    init_i2c();

    // Initialize BMP280
    sensor = bmx280_create(I2C_MASTER_NUM);
    if (!sensor) {
        ESP_LOGE(TAG, "Failed to create BMP280 instance");
    }

    ESP_ERROR_CHECK(bmx280_init(sensor));

    bmx280_config_t cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(sensor, &cfg));

    if (sensor) {
        ESP_LOGI(TAG, "BMP280 init success");
    } else {
        ESP_LOGE(TAG, "BMP280 init failed");
        return;
    }

    ESP_ERROR_CHECK(bmx280_setMode(sensor, BMX280_MODE_CYCLE));
}

void accel_task(void *duty_cycle_ms) {
    init_bmp280();

    int32_t temp;
    uint32_t pres;
    while (1) {
        // Wait for sensor to finish sampling
        while (bmx280_isSampling(sensor)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        ESP_ERROR_CHECK(bmx280_readout(sensor, &temp, &pres, NULL));

        ESP_LOGI(TAG, "Read Values: temp = %2d °C, pres = %.2f Pa", temp, (float)(pres / 256.0f));

        vTaskDelay(pdMS_TO_TICKS((uint32_t)duty_cycle_ms));
    }
    bmx280_close(sensor);
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGW(TAG, "Leaving Accel Task");
}

static tsl2591_t light_sensor_dev;
static void init_light_sensor(void) {
    // Ensure I2C is stated
    init_i2c();

    ESP_ERROR_CHECK(tsl2591_init_desc(&light_sensor_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(tsl2591_init(&light_sensor_dev));

    ESP_ERROR_CHECK(tsl2591_set_power_status(&light_sensor_dev, TSL2591_POWER_ON));
    ESP_ERROR_CHECK(tsl2591_set_als_status(&light_sensor_dev, TSL2591_ALS_ON));

    ESP_ERROR_CHECK(tsl2591_set_gain(&light_sensor_dev, TSL2591_GAIN_MEDIUM));
    ESP_ERROR_CHECK(tsl2591_set_integration_time(&light_sensor_dev, TSL2591_INTEGRATION_300MS));
}

void light_sensor_task(void *duty_cycle_ms) {
    init_light_sensor();

    float lux;
    esp_err_t res;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if ((res = tsl2591_get_lux(&light_sensor_dev, &lux)) == ESP_OK) {
            ESP_LOGI("Light", "Lux reading: %f", lux);
        } else {
            ESP_LOGW("Light", "Cound not read Lux value: %d", res);
        }
    }
}
