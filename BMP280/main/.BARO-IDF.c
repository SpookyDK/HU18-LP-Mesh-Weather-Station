#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO           11
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

static const char *TAG = "MPU6050_EXAMPLE";

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

    while (1) {

        mpu6050_acce_value_t accel;
        mpu6050_gyro_value_t gyro;
        mpu6050_temp_value_t temp;
        complimentary_angle_t angle;

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

        ESP_LOGI(TAG,
                 "ACC[g]: X=%.2f Y=%.2f Z=%.2f | "
                 "GYRO[dps]: X=%.2f Y=%.2f Z=%.2f | "
                 "Temp=%.2f°C | "
                 "Roll=%.2f Pitch=%.2f",
                 accel.acce_x, accel.acce_y, accel.acce_z,
                 gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                 temp.temp,
                 angle.roll, angle.pitch);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}