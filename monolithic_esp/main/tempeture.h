#ifndef TEMPETURE
#define TEMPETURE

void temp_task(void *duty_cycle_ms);

#define CONFIG_DHT11_PIN GPIO_NUM_12
#define CONFIG_CONNECTION_TIMEOUT 5

#define ONEWIRE_BUS_GPIO 22
#define ONEWIRE_MAX_DEVS 4

#endif // !TEMPETURE
