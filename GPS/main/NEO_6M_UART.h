#pragma once
#include "freertos/idf_additions.h"
#include <stdint.h>

#define GPS_UART_NUM UART_NUM_1
#define GPS_RX_PIN 1
#define GPS_TX_PIN 2
#define BUF_SIZE 2048
#include <stdint.h> // needed for uint32_t, uint16_t, etc.

int gpsCalcCheckSum(uint8_t *sentence, uint8_t messageLengthInc);
int gpsSendMessage(uint8_t *sentencte, uint8_t messageLengthInc);
void gpsInitUart();
void gpsTask();
int gpsSaveHotstartData();
static uint8_t data[1024];
