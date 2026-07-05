#include "leaf_sensor.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

/* raw response for debug — readable via OpenOCD */
volatile uint8_t leaf_raw_rx[9] = {0};

/* ──────────────────────────────────────────────────────────
 * ModBus CRC16 (polynomial 0x8005, reversed → 0xA001)
 * Calculated over [addr, func, ..., last_data_byte]
 * Result transmitted low-byte first.
 * ────────────────────────────────────────────────────────── */
static uint16_t modbus_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ──────────────────────────────────────────────────────────
 * Read leaf sensor (default address 0x01 @ 4800 bps)
 *
 * Query:  [01] [03] [00 00] [00 02] [CRC]
 * Response: [01] [03] [04] [T_H T_L] [H_H H_L] [CRC]
 *
 * Temperature: signed 16-bit, raw/10 = °C, we store in 0.01 °C (×10)
 * Humidity:    unsigned 16-bit, raw/10 = %RH, we store in 0.01 %RH (×10)
 * ────────────────────────────────────────────────────────── */
bool leaf_sensor_read(int16_t *temp, uint16_t *humidity)
{
    if (temp == NULL || humidity == NULL) {
        return false;
    }

    /* ── Build ModBus query ── */
    uint8_t tx[8];
    tx[0] = 0x01;   /* device address */
    tx[1] = 0x03;   /* function: read holding registers */
    tx[2] = 0x00;   /* start addr high */
    tx[3] = 0x00;   /* start addr low  (0x0000 = temperature) */
    tx[4] = 0x00;   /* quantity high */
    tx[5] = 0x02;   /* quantity low   (read 2 registers) */

    uint16_t crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc);       /* CRC low  byte */
    tx[7] = (uint8_t)(crc >> 8);  /* CRC high byte */

    /* ── Send query (blocking) ── */
    if (HAL_UART_Transmit(&huart1, tx, 8, 100) != HAL_OK) {
        return false;
    }

    /* ── Receive response: 9 bytes total ── */
    uint8_t rx[9];
    if (HAL_UART_Receive(&huart1, rx, 9, 200) != HAL_OK) {
        return false;
    }

    /* save raw response for debug */
    for (int i = 0; i < 9; i++) leaf_raw_rx[i] = rx[i];

    /* ── Sanity checks ── */
    if (rx[0] != 0x01 ||   /* address */
        rx[1] != 0x03 ||   /* function */
        rx[2] != 0x04) {   /* byte count */
        return false;
    }

    /* ── CRC verification (over bytes 0..6) ── */
    crc = modbus_crc16(rx, 7);
    uint16_t rx_crc = (uint16_t)rx[7] | ((uint16_t)rx[8] << 8);
    if (crc != rx_crc) {
        return false;
    }

    /* ── Parse sensor values ──
     * Raw values are in 0.1 units; internal format uses 0.01 units.
     * Multiply by 10 to convert: 25.1 °C → 2510  (internal)
     */
    int16_t raw_temp = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
    uint16_t raw_hum = (uint16_t)((rx[5] << 8) | rx[6]);

    *temp     = raw_temp * 10;
    *humidity = raw_hum * 10;

    return true;
}
