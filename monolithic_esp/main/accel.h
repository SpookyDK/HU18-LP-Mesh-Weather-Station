#ifndef ACCEL
#define ACCEL

void accel_task(void *duty_cycle_ms);

#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#endif // !ACCEL
