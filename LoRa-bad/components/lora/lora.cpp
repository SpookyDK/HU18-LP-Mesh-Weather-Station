#include "lora.h"

#define LORA_SPI_FREQ      1000000
#define LORA_SPI_WNR       0x80

LORA::LORA(gpio_num_t nss, gpio_num_t reset,
           gpio_num_t txEn, gpio_num_t rxEn)
{
    _nssPin = nss;
    _resetPin = reset;
    _txEnPin = txEn;
    _rxEnPin = rxEn;
}

void LORA::pinInit()
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << _nssPin) |
        (1ULL << _resetPin) |
        (1ULL << _txEnPin) |
        (1ULL << _rxEnPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    gpio_set_level(_nssPin, 1);
    gpio_set_level(_txEnPin, 0);
    gpio_set_level(_rxEnPin, 0);
}

void LORA::spiInit(spi_host_device_t host)
{
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = GPIO_NUM_19;
    buscfg.mosi_io_num = GPIO_NUM_23;
    buscfg.sclk_io_num = GPIO_NUM_18;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 256;

    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = LORA_SPI_FREQ;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;  // manual CS
    devcfg.queue_size = 1;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;

    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &spi));
}

bool LORA::init(spi_host_device_t host)
{
    pinInit();
    spiInit(host);
    powerOnReset();
    return true;
}

void LORA::powerOnReset()
{
    gpio_set_level(_txEnPin, 0);
    gpio_set_level(_rxEnPin, 0);

    gpio_set_level(_resetPin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(_resetPin, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void LORA::select()
{
    gpio_set_level(_nssPin, 0);
}

void LORA::deselect()
{
    gpio_set_level(_nssPin, 1);
}

uint8_t LORA::SPIReadReg(uint8_t addr)
{
    spi_transaction_t t = {};
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr & 0x7F;
    tx[1] = 0x00;

    memset(rx, 0, sizeof(rx));

    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    select();
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    deselect();

    return rx[1];
}

void LORA::SPIWriteReg(uint8_t addr, uint8_t value)
{
    spi_transaction_t t = {};
    uint8_t tx[2];

    tx[0] = addr | LORA_SPI_WNR;
    tx[1] = value;

    t.length = 16;
    t.tx_buffer = tx;

    select();
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    deselect();
}

void LORA::SPIBurstWrite(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (len == 0) return;

    uint8_t buffer[256];

    buffer[0] = addr | LORA_SPI_WNR;
    memcpy(&buffer[1], data, len);

    spi_transaction_t t = {};
    t.length = (len + 1) * 8;
    t.tx_buffer = buffer;

    select();
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    deselect();
}

void LORA::SPIBurstRead(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (len == 0) return;

    uint8_t tx[256];
    uint8_t rx[256];

    memset(tx, 0, len + 1);
    tx[0] = addr & 0x7F;

    spi_transaction_t t = {};
    t.length = (len + 1) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    select();
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    deselect();

    memcpy(data, &rx[1], len);
}

//temp fix
bool LORA::txPacket(uint8_t* data, uint8_t len)
{
    SPIBurstWrite(0x00, data, len);
    return true;
}