#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// SPI
#define SPI_MISO_GPIO 26
#define SPI_MOSI_GPIO 27
#define SPI_SCK_GPIO 4

// LoRa
#define LORA_CS_GPIO 9

#define GPS_DISABLE_PIN 25
// Note these are from the perspective of the ESP
// Meaning that TX shall connect to RX on the GPS
#define GPS_TX_PIN 2
#define GPS_RX_PIN 3

// Soil Temperature sensor pins
#define ONEWIRE_BUS_GPIO 22
#define ONEWIRE_MAX_DEVS 4
// DHT11 data pin
#define CONFIG_DHT11_PIN GPIO_NUM_12

// PCNT sensor pins
#define WIND_PCNT_INPUT_PIN 1
#define RAIN_PCNT_INPUT_PIN 0

// Moist sensor pin
#define MOISTURE_ADC_CHANNEL ADC_CHANNEL_4 // This is GPIO 5, on the ESP32-H2

// I2C pins
#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_SDA_IO 10

// Otehr config
#define PACKET_TIME_TO_LIVE 3

// NVS keys
#define NVS_NAMESPACE "lora_conf"
#define NVS_NETWORK_KEY "network_key"
#define NVS_NODE_KEY "node_key"
#define NVS_PACKET_ID_KEY "packet_key"

#endif // PIN_CONFIG_H
