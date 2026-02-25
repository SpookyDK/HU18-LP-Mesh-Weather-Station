#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG = "LORA_MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting LoRa...");

    // Adjust pins to your wiring
    LORA lora(
        GPIO_NUM_5,   // NSS (CS)
        GPIO_NUM_14,  // RESET
        GPIO_NUM_26,  // TX_EN
        GPIO_NUM_27   // RX_EN
    );

    // Initialize SPI on VSPI
    lora.init(SPI2_HOST);

    ESP_LOGI(TAG, "LoRa Initialized");

    uint8_t message[] = "WeatherNode_01";

    while (true)
    {
        ESP_LOGI(TAG, "Sending packet...");
        lora.txPacket(message, sizeof(message) - 1);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}