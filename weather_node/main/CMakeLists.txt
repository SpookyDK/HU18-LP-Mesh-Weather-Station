idf_component_register(SRCS "main.c" "moist_soil.c" "tempeture.c" "i2c_tasks.c" "my_drivers/bmp280.c" "my_drivers/lora.c" "pcnt_sensor.c" "my_drivers/NEO_6M_UART.c" 
                       PRIV_REQUIRES dht11 ds18b20 onewire_bus tsl2591
                       INCLUDE_DIRS "." "my_drivers" "packet_def"
                       REQUIRES nvs_flash driver esp_driver_gpio esp_adc)

if(ENABLE_ANALYZER)
target_compile_options(${COMPONENT_LIB} PRIVATE 
    "-fanalyzer"           # The logic/path analyzer
    # "-Wanalyzer-too-complex" # Warns if the analyzer gives up on a complex function
    "-Wall"                # Standard good warnings
    "-Wextra"              # Extra "suspicious" code warnings
    "-Wshadow"             # Warns if you name a local variable the same as a global
    "-Wdouble-promotion"   # CRITICAL for ESP32: warns if you accidentally use double math on a float CPU
    "-Wnull-dereference"   # Additional check for null pointers outside the analyzer
    "-Wstack-usage=2048"   # Warns if a function uses more than 2KB of stack (prevents stack overflows)
)
endif()
