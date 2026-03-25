#ifndef I2C_TASKS
#define I2C_TASKS

#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

void barometer_task(void *duty_cycle_ms);
void light_sensor_task(void *duty_cycle_ms);
void power_sensor_task(void *sensor_cnt);

#endif // !I2C_TASKS
