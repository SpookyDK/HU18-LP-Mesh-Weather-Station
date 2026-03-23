#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#define LORA_MISO_GPIO 26
#define LORA_MOSI_GPIO 27
#define LORA_SCK_GPIO 4
#define LORA_CS_GPIO 9

#define GPS_DISABLE_PIN 25
// Note these are from the perspective of the ESP
// Meaning that TX shall connect to RX on the GPS
#define GPS_TX_PIN 2
#define GPS_RX_PIN 3

#endif // PIN_CONFIG_H
