#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#define SPI_MISO_GPIO 5
#define SPI_MOSI_GPIO 4
#define SPI_SCK_GPIO 6

#define LORA_CS_GPIO 3
#define LORA_DIO0_GPIO 2

//  Note these are from the perspective of the ESP
//  Meaning that TX shall connect to RX on the GPS
#define GPS_TX_PIN 19
#define GPS_RX_PIN 18

#endif // !PIN_CONFIG_H
