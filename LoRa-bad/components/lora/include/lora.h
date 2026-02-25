#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

class LORA {
public:
    LORA(gpio_num_t nss, gpio_num_t reset,
         gpio_num_t txEn, gpio_num_t rxEn);

    bool init(spi_host_device_t host);
    bool txPacket(uint8_t* data, uint8_t len);
    uint8_t rxPacket(uint8_t* buffer);

private:
    gpio_num_t _nssPin;
    gpio_num_t _resetPin;
    gpio_num_t _txEnPin;
    gpio_num_t _rxEnPin;

    spi_device_handle_t spi;

    void spiInit(spi_host_device_t host);
    void pinInit();
    void powerOnReset();

    uint8_t SPIReadReg(uint8_t addr);
    void SPIWriteReg(uint8_t addr, uint8_t value);
    void SPIBurstWrite(uint8_t addr, uint8_t *data, uint8_t len);
    void SPIBurstRead(uint8_t addr, uint8_t *data, uint8_t len);

    void select();
    void deselect();
};