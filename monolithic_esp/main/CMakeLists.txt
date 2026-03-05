idf_component_register(SRCS "main.c" "NEO_6M_UART.c" "moist_soil.c" "tempeture.c" "accel.c"
                       PRIV_REQUIRES dht11 ds18b20 onewire_bus bmx280
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash driver esp_driver_gpio esp_adc)
