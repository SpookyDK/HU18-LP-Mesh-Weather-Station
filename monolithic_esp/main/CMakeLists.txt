idf_component_register(SRCS "main.c" "NEO_6M_UART.c" "moist_soil.c" "tempeture.c" "i2c_tasks.c" "my_drivers/bmp280.c"
                       PRIV_REQUIRES dht11 ds18b20 onewire_bus tsl2591
                       INCLUDE_DIRS "." "my_drivers"
                       REQUIRES nvs_flash driver esp_driver_gpio esp_adc)
