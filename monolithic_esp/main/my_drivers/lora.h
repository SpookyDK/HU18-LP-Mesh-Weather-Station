#ifndef __LORA_H__
#define __LORA_H__

// Pins
#define CONFIG_MISO_GPIO 26
#define CONFIG_MOSI_GPIO 27
#define CONFIG_SCK_GPIO 4
#define CONFIG_CS_GPIO 9

#include <stdint.h>

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void lora_write_reg(int reg, int val);

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int lora_read_reg(int reg);

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void);

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(void);

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(int size);

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_idle(void);

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(void);

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_receive(void);

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void lora_set_tx_power(int level);

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency);

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int sf);

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void lora_set_bandwidth(long sbw);

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void lora_set_coding_rate(int denominator);

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(long length);

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void lora_set_sync_word(int sw);

/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void);

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(void);

/**
 * Perform hardware initialization.
 */
int lora_init(void);

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, int size);

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int lora_receive_packet(uint8_t *buf, int size);

/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(void);

/**
 * Blocks task if LoRa is receiving
 */
void block_if_receiving(void);

/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(void);

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(void);

/**
 * Shutdown hardware.
 */
void lora_close(void);
void lora_dump_registers(void);

#endif
