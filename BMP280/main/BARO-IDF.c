#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "bmx280.h"
#include <math.h>

#define SEA_LEVEL_PRESSURE_PA 99120.0f  // adjust for your local sea level pressure
#define I2C_MASTER_NUM I2C_NUM_0
#define BMX280_SDA_NUM 10   // Change according to your board
#define BMX280_SCL_NUM 11   // Change according to your board
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "BMP280_EXAMPLE";

// Initialize I2C using standard ESP-IDF driver
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BMX280_SDA_NUM,
        .scl_io_num = BMX280_SCL_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// Initialize BMP280 using the utkumaden BMX280 library
static bmx280_t* bmx280_dev_init(void)
{
    bmx280_t *dev = bmx280_create(I2C_MASTER_NUM);
    if (!dev) {
        ESP_LOGE(TAG, "Failed to create BMP280 instance");
        return NULL;
    }

    ESP_ERROR_CHECK(bmx280_init(dev));

    bmx280_config_t cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(dev, &cfg));

    return dev;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BMP280 Example");

    // Initialize I2C
    i2c_master_init();

    // Initialize BMP280
    bmx280_t *sensor = bmx280_dev_init();
    if (!sensor) {
        ESP_LOGE(TAG, "BMP280 init failed");
        return;
    }else if(sensor){
        ESP_LOGE(TAG, "BMP280 init success");
    }

    ESP_ERROR_CHECK(bmx280_setMode(sensor, BMX280_MODE_CYCLE));

    float temp = 0, pres = 0, hum = 0;


    while (1) {
        // Wait for sensor to finish sampling
        while (bmx280_isSampling(sensor)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        ESP_ERROR_CHECK(bmx280_readoutFloat(sensor, &temp, &pres, &hum));

        ESP_LOGI(TAG,
                 "Read Values: temp = %.2f °C, pres = %.2f Pa",
                 temp, pres);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bmx280_close(sensor);

    // Cleanup I2C driver
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));

    ESP_LOGI(TAG, "Restarting now.");
    esp_restart();
}
