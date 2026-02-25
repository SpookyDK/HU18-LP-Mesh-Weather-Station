idf_component_register(SRCS "main.c" "NEO_6M_UART.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash driver esp_driver_gpio dht11 ds18b20 onewire_bus )
