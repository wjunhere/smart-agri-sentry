#ifndef __SOIL_SENSOR_H
#define __SOIL_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Soil sensor data (8 ModBus registers starting from 0x0000)
 *
 *         Register map (confirmed from official STM32 TTL example):
 *           0x0000 : humidity      (unsigned 16-bit, /10 → %RH)
 *           0x0001 : temperature   (signed 16-bit,   /10 → °C)
 *           0x0002 : EC            (unsigned 16-bit, us/cm)
 *           0x0003 : pH            (unsigned 16-bit, /10 → pH)
 *           0x0004 : nitrogen N    (unsigned 16-bit, mg/kg)
 *           0x0005 : phosphorus P  (unsigned 16-bit, mg/kg)
 *           0x0006 : potassium K   (unsigned 16-bit, mg/kg)
 *           0x0007 : salinity/TDS  (unsigned 16-bit, ppm)
 */
typedef struct {
    int16_t  temp;       /* 0.01 °C  internal format */
    uint16_t humidity;   /* 0.01 %RH internal format */
    uint16_t ec;         /* us/cm                   */
    uint16_t salt;       /* ppm (salinity/TDS)      */
    uint16_t n;          /* nitrogen  mg/kg         */
    uint16_t p;          /* phosphorus mg/kg        */
    uint16_t k;          /* potassium mg/kg         */
    uint16_t ph;         /* pH * 10   (e.g. 66 → 6.6) */
} soil_data_t;

/**
 * @brief  Read soil sensor at default address 0x02.
 */
bool soil_sensor_read(soil_data_t *out);

/**
 * @brief  Read soil sensor at a specific ModBus address.
 * @param  addr  ModBus slave address (e.g. 0x01, 0x02, 0x03).
 * @param  out   Pointer to soil_data_t to fill.
 * @return true on success.
 */
bool soil_sensor_read_addr(uint8_t addr, soil_data_t *out);

/* ── Diagnostic globals (readable via OpenOCD) ── */
extern volatile uint8_t  soil_raw_rx[21];    /* raw ModBus response */
extern volatile uint8_t  soil_raw_tx[8];     /* last query sent */
extern volatile uint8_t  soil_rx_count;      /* bytes actually received */
extern volatile uint8_t  soil_tx_ok;         /* 1 = transmit succeeded */
extern volatile uint8_t  soil_rx_status;     /* HAL receive status code */
extern volatile uint8_t  soil_diag_state;    /* diagnostic step (0-11) */
extern volatile uint16_t soil_crc_calc;      /* computed CRC for verification */

#endif
