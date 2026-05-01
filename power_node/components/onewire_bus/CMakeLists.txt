set(srcs "src/onewire_bus_api.c"
         "src/onewire_crc.c"
         "src/onewire_bus_impl_rmt.c"
         "src/onewire_device.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include" "interface"
                       PRIV_REQUIRES "esp_driver_rmt" "esp_driver_gpio")
