/**
 * Lora Mesh basic endnode (might not work)
**/

#include "endnode.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdbool.h>
#include <driver/gpio.h>

static const char *TAG = "NODE";

// State of Node should be private
static spi_device_handle_t spi;
static uint8_t rx_buffer[256]; // biggest possible buffer size for LoRa in one go
static uint8_t g_node_id = 0x00;
static uint16_t g_network_id = 0x0000;
static bool g_configured = false; // Has it been configured/paired with a gateway?

// CRC 16 functions with a polynomial of 0x1021 Init is 0xFFFF and we do not reflect. (borrowed)
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// NVS helpers
static void nvs_load_config(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "No NVS config found- going into pairing mode");
        g_configured = false;
        return;
    }
    uint8_t configured = 0;
    nvs_get_u8(h, NVS_KEY_CONFIGURED, &configured);
    if (configured) {
        nvs_get_u16(h, NVS_KEY_NETWORK_ID, &g_network_id);
        g_configured = true;
        ESP_LOGI(TAG, "Loaded NETWORK_ID=0x%04X from flash", g_network_id);
    } else {
        g_configured = false;
        ESP_LOGI(TAG, "Not configured — entering pairing mode");
    }
    nvs_close(h);
}

static void nvs_save_network_id(uint16_t network_id) {
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));
    nvs_set_u16(h, NVS_KEY_NETWORK_ID, network_id);
    nvs_set_u8(h, NVS_KEY_CONFIGURED, 1);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved NETWORK_ID=0x%04x successfully to flash", network_id);
}

// SPI read and write
static uint8_t spi_read_reg(uint8_t reg) {
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return rx[1];
}

static void spi_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg | 0x80, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

static void spi_write_buf(uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t tx[257];
    tx[0] = reg | 0x80;
    memcpy(&tx[1], buf, len);
    spi_transaction_t t = { .length = (len + 1) * 8, .tx_buffer = tx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

static void spi_read_buf(uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t tx[257]    = { 0 };
    uint8_t rxbuf[257] = { 0 };
    tx[0] = reg & 0x7F;
    spi_transaction_t t = { .length = (len + 1) * 8, .tx_buffer = tx, .rx_buffer = rxbuf };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    memcpy(buf, &rxbuf[1], len);
}

// SPI 
static void init_spi(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000,
        .mode           = 0,
        .spics_io_num   = PIN_NUM_NSS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

// Initilize SX1276
static bool sx1276_init(void) {
    uint8_t version = spi_read_reg(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1276 not alive! REG_VERSION=0x%02X (expected 0x12)", version);
        return false;
    }
    ESP_LOGI(TAG, "IT IS ALIVE! (sx1276) - version=0x%02X", version);
    
    spi_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_write_reg(REG_FRF_MSB, FRF_MSB_868_1);
    spi_write_reg(REG_FRF_MID, FRF_MID_868_1);
    spi_write_reg(REG_FRF_LSB, FRF_LSB_868_1);

    spi_write_reg(REG_PA_CONFIG, 0x8F); // PA boost max power
    spi_write_reg(REG_LNA,       0x23); // HF Boost

    spi_write_reg(REG_MODEM_CONFIG_1, LORA_MODEM_CONFIG1);
    spi_write_reg(REG_MODEM_CONFIG_2, LORA_MODEM_CONFIG2);
    spi_write_reg(REG_MODEM_CONFIG_3, LORA_MODEM_CONFIG3);

    spi_write_reg(REG_PREAMBLE_MSB, 0x00);
    spi_write_reg(REG_PREAMBLE_LSB, 0x08); // 8 symbols

    spi_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    spi_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    spi_write_reg(REG_DETECTION_OPTIMIZE,  0x03);
    spi_write_reg(REG_DETECTION_THRESHOLD, 0x0A);

    spi_write_reg(REG_SYNC_WORD, LORA_SYNC_WORD);
    spi_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    ESP_LOGI(TAG, "SX1276 ready - 868.1MHz, SF12, BW 125kHz, CR 4/5, SyncWord 0x%02X", LORA_SYNC_WORD);
    return true;
}

// RX TX
static void lora_start_rx(void) {
    spi_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    spi_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

static void lora_transmit(const uint8_t *data, size_t len) {
    spi_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    spi_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    spi_write_buf(REG_FIFO, data, len);
    spi_write_reg(REG_PAYLOAD_LENGTH, len);
    spi_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    uint32_t start = esp_timer_get_time() / 1000;
    while (!(spi_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
        if ((esp_timer_get_time() / 1000) - start > 5000) {
            ESP_LOGE(TAG, "TX timeout");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    spi_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE);
    lora_start_rx();
}

// PUBLIC: fill shit up, then get header + crc and transmit shit
void node_send_sensor_data(mesh_packet_t *pkt) {
    pkt->header.node_id    = g_node_id;
    pkt->header.network_id = g_network_id;
    pkt->header.orig_id    = g_node_id;
    pkt->header.hop_count  = 0;
    pkt->crc16      = crc16((uint8_t *)pkt, MESH_PACKET_SIZE - sizeof(uint16_t));

    ESP_LOGI(TAG, "TX node=0x%02x net=0x%04x", g_node_id, g_network_id);
    lora_transmit((uint8_t *)pkt, MESH_PACKET_SIZE);
}

// If we need to relay
static void relay_sensor_packet(mesh_packet_t *pkt) {
    if (pkt->header.hop_count >= MAX_RELAY_HOPS) {
        ESP_LOGD(TAG, "Not relaying — hop_count=%d", pkt->header.hop_count);
        return;
    }
    pkt->header.node_id = g_node_id;
    pkt->header.hop_count++;
    pkt->crc16 = crc16((uint8_t *)pkt, MESH_PACKET_SIZE - sizeof(uint16_t));

    ESP_LOGI(TAG, "RELAY orig=0x%02X hop=%d", pkt->header.orig_id, pkt->header.hop_count);
    lora_transmit((uint8_t *)pkt, MESH_PACKET_SIZE);
}

// pairing TX
static void send_pair_packet(uint8_t pkt_type, uint16_t network_id) {
    pair_packet_t pkt = {
        .pkt_type   = pkt_type,
        .network_id = network_id,
    };
    lora_transmit((uint8_t *)&pkt, sizeof(pkt));
}

// Handling pair beacon - save network ID and reply with ACK then reboot
static void handle_pair_beacon(const pair_packet_t *pkt) {
    ESP_LOGI(TAG, "PAIR BEACON — NETWORK_ID=0x%04X", pkt->network_id);

    nvs_save_network_id(pkt->network_id);
    g_network_id = pkt->network_id;

    send_pair_packet(PKT_TYPE_PAIR_ACK, pkt->network_id);

    ESP_LOGI(TAG, "Pairing complete — rebooting");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

// RX Handler - polls every 10ms roughly
static void lora_check_rx(void) {
    uint8_t irq = spi_read_reg(REG_IRQ_FLAGS);

    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        ESP_LOGW(TAG, "LoRa CRC error");
        spi_write_reg(REG_IRQ_FLAGS, 0xFF);
        return;
    }
    if (!(irq & IRQ_RX_DONE)) return;

    spi_write_reg(REG_IRQ_FLAGS, IRQ_RX_DONE);

    uint8_t len = spi_read_reg(REG_RX_NB_BYTES);
    uint8_t ptr = spi_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    spi_write_reg(REG_FIFO_ADDR_PTR, ptr);
    spi_read_buf(REG_FIFO, rx_buffer, len);

    // RSSI correction depends on SNR — datasheet 4.4
    // SNR >= 0:  RSSI = -157 + RegPktRssiValue
    // SNR <  0:  RSSI = -157 + RegPktRssiValue + RegPktSnrValue / 4
    int8_t snr  = (int8_t)spi_read_reg(REG_PKT_SNR_VALUE);
    int8_t rssi = -157 + (int8_t)spi_read_reg(REG_PKT_RSSI_VALUE);
    if (snr < 0) rssi += snr / 4;

    // pair mode
    if (!g_configured) {
        if (len == sizeof(pair_packet_t)) {
            pair_packet_t *pair = (pair_packet_t *)rx_buffer;
            if (pair->pkt_type == PKT_TYPE_PAIR_BEACON)
                handle_pair_beacon(pair);
        } else {
            ESP_LOGD(TAG, "Pairing mode — ignoring non-beacon packet");
        }
        return;
    }

    // Operational mode
    if (len != MESH_PACKET_SIZE) {
        ESP_LOGD(TAG, "Unexpected size %d — ignoring", len);
        return;
    }

    mesh_packet_t *pkt = (mesh_packet_t *)rx_buffer;

    if (pkt->header.network_id != g_network_id) {
        ESP_LOGD(TAG, "not my network 0x%04X — dropping", pkt->header.network_id);
        return;
    }
    if (pkt->header.orig_id == g_node_id) {
        ESP_LOGD(TAG, "echo — ignoring");
        return;
    }

    // Verify CRC-16
    uint16_t expected = crc16((uint8_t *)pkt, MESH_PACKET_SIZE - sizeof(uint16_t));
    if (pkt->crc16 != expected) {
        ESP_LOGW(TAG, "CRC-16 mismatch got=0x%04X expected=0x%04X",
                 pkt->crc16, expected);
        return;
    }

    ESP_LOGI(TAG, "RX %d bytes RSSI=%ddBm", len, rssi);
    relay_sensor_packet(pkt);
}

// Node ID from factory MAC address (EUI-64)
static void init_node_id(void) {
    uint8_t mac[8];
    esp_read_mac(mac, ESP_MAC_IEEE802154);
    g_node_id = mac[7];
    if (g_node_id == 0xFF) g_node_id = 0xFE;
    if (g_node_id == 0x00) g_node_id = 0x01;
    ESP_LOGI(TAG, "NODE ID: 0x%02X (MAC ...%02X:%02X:%02X)", g_node_id, mac[5], mac[6], mac[7]);
}

// DEMO STUFF MIGHT WORK
static void send_demo_data(void) {
    mesh_packet_t pkt = {
        .payload.longitude    = 102039000,  // 10.2039000° E
        .payload.latitude     = 561629000,  // 56.1629000° N
        .payload.air_humidity = 72,
        .payload.air_temperature     = 215,       
        .payload.soil_temperature    = { 537, 540, 535, 538 }, 
        .payload.soil_moisture = 65,
        .payload.pressure     = 1013,       
        .payload.lux          = 45000,
        .payload.perceptation = 0,
        .payload.wind_speed   = 1500, 
    };
    node_send_sensor_data(&pkt);
}
// DEBUG
static uint8_t bitbang_read_reg(uint8_t reg) {
    uint8_t value = 0;
    reg &= 0x7F;  // read bit

    gpio_set_level(PIN_NUM_NSS, 0);
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_NUM_SCK, 0);
        gpio_set_level(PIN_NUM_MOSI, (reg >> i) & 1);
        gpio_set_level(PIN_NUM_SCK, 1);
    }
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_NUM_SCK, 0);
        gpio_set_level(PIN_NUM_MOSI, 0);
        gpio_set_level(PIN_NUM_SCK, 1);
        value |= gpio_get_level(PIN_NUM_MISO) << i;
    }
    gpio_set_level(PIN_NUM_NSS, 1);
    return value;
}

void bitbang_test(void) {
    gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_SCK,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_NSS,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);
    gpio_set_level(PIN_NUM_NSS, 1);
    gpio_set_level(PIN_NUM_SCK, 0);

    ESP_LOGI(TAG, "Bitbang test:");
    ESP_LOGI(TAG, "  VERSION  = 0x%02X", bitbang_read_reg(0x42));
    ESP_LOGI(TAG, "  OP_MODE  = 0x%02X", bitbang_read_reg(0x01));
}
// DEBUG END

// Main 
void app_main(void) {
    init_node_id();
    ESP_LOGI(TAG, "=== LoRa Weather Node 0x%02X booting ===", g_node_id);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    nvs_load_config();
    bitbang_test(); // REMOVE LATER DEBUG
    init_spi();

    if (!sx1276_init()) {
        ESP_LOGE(TAG, "SX1276 init failed — halting");
        return;
    }

    lora_start_rx();

    if (!g_configured) {
        ESP_LOGI(TAG, "PAIRING MODE — power on gateway and click 'Add nodes'");
        while (1) {
            lora_check_rx();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "OPERATIONAL — network=0x%04X", g_network_id);

    uint32_t last_tx_ms        = 0;
    const uint32_t TX_INTERVAL = 60000;

    while (1) {
        lora_check_rx();

        uint32_t now_ms = esp_timer_get_time() / 1000;
        if ((now_ms - last_tx_ms) >= TX_INTERVAL) {
            send_demo_data();
            last_tx_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}