#ifndef MOIST_SOIL
#define MOIST_SOIL

#define MOISTURE_ADC_CHANNEL ADC_CHANNEL_4 // This is GPIO 5, on the ESP32-H2
#define MOISTURE_ADC_ATTEN ADC_ATTEN_DB_12

void moist_task(void *duty_cycle_ms);

#endif // !MOIST_SOIL
