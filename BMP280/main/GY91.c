#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "bmx280.h"
#include <math.h>

#define I2C_MASTER_SCL_IO           11
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

static const char *TAG = "GY91 test";

/* ---------------- I2C INIT ---------------- */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


/*SPLITTER FOR BMP280*/
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


/* ---------------- MAIN ---------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    // Create sensor handle
    mpu6050_handle_t mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu == NULL) {
        ESP_LOGE(TAG, "Failed to create MPU6050");
        return;
    }

    // Check device ID //COULD DELETE AS NOT NEEDED
    uint8_t device_id = 0;
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu, &device_id));
    ESP_LOGI(TAG, "Device ID: 0x%02X", device_id);

    if (device_id != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU6050 not detected!");
        return;
    }

    // Wake up sensor
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));

    // Configure full scale ranges
    ESP_ERROR_CHECK(
        mpu6050_config(
            mpu,
            ACCE_FS_2G,
            GYRO_FS_250DPS
        )
    );

    ESP_LOGI(TAG, "MPU6050 Initialized Successfully");

    // Initialize BMP280
    bmx280_t *sensor = bmx280_dev_init();
    if (!sensor) {
        ESP_LOGE(TAG, "BMP280 init failed");
        return;
    }else if(sensor){
        ESP_LOGE(TAG, "BMP280 init success");
    }

    ESP_ERROR_CHECK(bmx280_setMode(sensor, BMX280_MODE_CYCLE));

    float btemp = 0, bpres = 0, bhum = 0;
    
    mpu6050_acce_value_t accel;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t angle;

    while (1) {

        ESP_ERROR_CHECK(mpu6050_get_acce(mpu, &accel));
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, &gyro));
        ESP_ERROR_CHECK(mpu6050_get_temp(mpu, &temp));

        ESP_ERROR_CHECK(
            mpu6050_complimentory_filter(
                mpu,
                &accel,
                &gyro,
                &angle
            )
        );

        /*ESP_LOGI(TAG,
                 "ACC[g]: X=%.2f Y=%.2f Z=%.2f | "
                 "GYRO[dps]: X=%.2f Y=%.2f Z=%.2f | "
                 "Temp=%.2f°C | "
                 "Roll=%.2f Pitch=%.2f",
                 accel.acce_x, accel.acce_y, accel.acce_z,
                 gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                 temp.temp,
                 angle.roll, angle.pitch);*/
        
        // Wait for sensor to finish sampling
        while (bmx280_isSampling(sensor)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        ESP_ERROR_CHECK(bmx280_readoutFloat(sensor, &btemp, &bpres, &bhum));

        /*ESP_LOGI(TAG,
                 "Read Values: temp = %.2f °C, pres = %.2f Pa",
                 btemp, bpres);*/

        ESP_LOGI(TAG,
                 "Read Values: temp = %.2f °C, pres = %.2f Pa\nACC[g]: X=%.2f Y=%.2f Z=%.2f | "
                 "GYRO[dps]: X=%.2f Y=%.2f Z=%.2f | "
                 "Temp=%.2f°C | "
                 "Roll=%.2f Pitch=%.2f",
                 btemp, bpres,accel.acce_x, accel.acce_y, accel.acce_z,
                 gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                 temp.temp,
                 angle.roll, angle.pitch);

        vTaskDelay(pdMS_TO_TICKS(1000));
    
    }
    bmx280_close(sensor);

    // Cleanup I2C driver
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));

    ESP_LOGI(TAG, "Restarting now.");
    esp_restart();
}
