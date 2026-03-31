#ifndef I2C_TASKS
#define I2C_TASKS

#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10
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

void barometer_task(void *duty_cycle_ms);
void light_sensor_task(void *duty_cycle_ms);
void power_sensor_task(void *sensor_cnt);

#endif // !I2C_TASKS
