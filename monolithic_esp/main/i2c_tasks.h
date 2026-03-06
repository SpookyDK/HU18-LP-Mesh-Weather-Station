#ifndef I2C_TASKS
#define I2C_TASKS

void accel_task(void *duty_cycle_ms);
void light_sensor_task(void *duty_cycle_ms);

#endif // !I2C_TASKS
