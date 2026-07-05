#include "soil_sensor.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart4;

/* ──────────────────────────────────────────────────────────
 * ModBus CRC16 (polynomial 0xA001)
 * Identical to official STM32 TTL example: Calculate_CRC16()
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

/* ── Diagnostic globals ── */
volatile uint8_t soil_raw_rx[21]   = {0};
volatile uint8_t soil_raw_tx[8]    = {0};   /* last query sent */
volatile uint8_t soil_rx_count     = 0;
volatile uint8_t soil_tx_ok        = 0;
volatile uint8_t soil_rx_status    = 0;
volatile uint8_t soil_diag_state   = 0;
volatile uint16_t soil_crc_calc    = 0;      /* computed CRC for verification */

/* ──────────────────────────────────────────────────────────
 * Read soil sensor at a specific ModBus address.
 *
 * Uses simple blocking approach matching the official
 * STM32 TTL example exactly:
 *   HAL_UART_Transmit(8 bytes) → HAL_UART_Receive(21 bytes, 1000ms)
 *
 * Register map (confirmed from official STM32 TTL example):
 *   0x0000 : humidity (%RH*10), 0x0001 : temperature (°C*10)
 *   0x0002 : EC (us/cm),       0x0003 : pH (*10)
 *   0x0004 : N (mg/kg),        0x0005 : P (mg/kg)
 *   0x0006 : K (mg/kg),        0x0007 : salinity (ppm)
 * ────────────────────────────────────────────────────────── */
bool soil_sensor_read_addr(uint8_t addr, soil_data_t *out)
{
    soil_diag_state = 0;

    if (out == NULL) {
        return false;
    }

    /* ── Build query: addr 03 00 00 00 08 CRC ── */
    uint8_t tx[8];
    tx[0] = addr;
    tx[1] = 0x03;
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x08;

    uint16_t crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc);        /* CRC low  byte */
    tx[7] = (uint8_t)(crc >> 8);   /* CRC high byte */

    /* Save TX for OpenOCD verification */
    for (int i = 0; i < 8; i++) soil_raw_tx[i] = tx[i];
    soil_crc_calc = crc;

    /* ── Clear RX buffer to 0xFF (sentinel: 0xFF = no data) ── */
    memset((void *)soil_raw_rx, 0xFF, sizeof(soil_raw_rx));
    soil_rx_count  = 0;
    soil_tx_ok     = 0;
    soil_rx_status = 0;

    soil_diag_state = 1;

    /* ── 1. Send query ── */
    HAL_StatusTypeDef tx_st = HAL_UART_Transmit(&huart4, tx, 8, 200);
    if (tx_st != HAL_OK) {
        soil_diag_state = 2;
        return false;
    }
    soil_tx_ok = 1;
    soil_diag_state = 3;

    /* ── 2. Short delay for sensor processing (50ms) ── */
    HAL_Delay(50);

    /* ── 3. Receive response (matches official example: 21 bytes, 1000ms) ── */
    uint8_t rx[21];
    memset(rx, 0, sizeof(rx));
    HAL_StatusTypeDef rx_st = HAL_UART_Receive(&huart4, rx, 21, 200);
    soil_rx_status = (uint8_t)rx_st;
    soil_diag_state = 4;

    /* Save whatever arrived */
    for (uint8_t i = 0; i < sizeof(soil_raw_rx); i++) {
        soil_raw_rx[i] = rx[i];
    }

    if (rx_st != HAL_OK) {
        /* Count non-zero bytes in rx buffer */
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 21; i++) {
            if (rx[i] != 0x00 && rx[i] != 0xFF) cnt++;
        }
        soil_rx_count = cnt;
        soil_diag_state = 5;
        return false;
    }
    soil_rx_count = 21;
    soil_diag_state = 6;

    /* ── Sanity checks ── */
    if (rx[0] != addr || rx[1] != 0x03 || rx[2] != 0x10) {
        soil_diag_state = 7;
        return false;
    }

    soil_diag_state = 9;

    /* ── CRC verification ── */
    crc = modbus_crc16(rx, 19);
    uint16_t rx_crc = (uint16_t)rx[19] | ((uint16_t)rx[20] << 8);
    if (crc != rx_crc) {
        soil_diag_state = 10;
        return false;
    }

    soil_diag_state = 11;

    /* ── Parse ── */
    out->humidity = ((uint16_t)((rx[3] << 8) | rx[4])) * 10;
    out->temp     = ((int16_t)(((uint16_t)rx[5] << 8) | rx[6])) * 10;
    out->ec       =  (uint16_t)((rx[7]  << 8) | rx[8]);
    out->ph       =  (uint16_t)((rx[9]  << 8) | rx[10]);
    out->n        =  (uint16_t)((rx[11] << 8) | rx[12]);
    out->p        =  (uint16_t)((rx[13] << 8) | rx[14]);
    out->k        =  (uint16_t)((rx[15] << 8) | rx[16]);
    out->salt     =  (uint16_t)((rx[17] << 8) | rx[18]);

    return true;
}

bool soil_sensor_read(soil_data_t *out)
{
    return soil_sensor_read_addr(0x02, out);
}
