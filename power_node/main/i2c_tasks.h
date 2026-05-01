#ifndef I2C_TASKS
#define I2C_TASKS

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define INA219_REG_CONFIG (0x00)                    ///< Config register
#define INA219_CONFIG_RESET (0x8000)                ///< Config reset register
#define INA219_CONFIG_BUSVOLTAGERANGE_MASK (0x2000) ///< Config bus voltage range
#define INA219_REG_SHUNTVOLTAGE (0x01)              ///< Shunt Voltage Register
#define INA219_REG_BUSVOLTAGE (0x02)                ///< Bus Voltage Register
#define INA219_REG_POWER (0x03)                     ///< Power Register
#define INA219_REG_CURRENT (0x04)                   ///< Current Register
#define INA219_REG_CALIBRATION (0x05)               ///< Register Calibration
#define INA219_CALIBRATION_VALUE 4000
#define INA219_VOLTAGE_LSB_MV 4 // 4mV per LSB
#define INA219_CURRENT_LSB_MA 1 // 1mA per LSB
#define INA219_POWER_LSB_MW 20  // 20x current LSB = 20mW per LSB

#define POWER_ADDR_SOLAR 0x41
#define POWER_ADDR_BATTERY 0x40

#include "esp_err.h"
#include "packet_def.h"

esp_err_t get_pres_read(sensor_payload_t *payload);
esp_err_t get_light_read(sensor_payload_t *payload);
esp_err_t get_power_read(sensor_payload_t *payload, uint8_t addr);

#endif // !I2C_TASKS
