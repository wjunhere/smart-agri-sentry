#ifndef __LEAF_SENSOR_H
#define __LEAF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Read leaf surface temperature and humidity via RS485 ModBus-RTU.
 * @param  temp     Output: leaf temperature in 0.01 °C (signed)
 * @param  humidity Output: leaf wetness/humidity in 0.01 %RH (unsigned)
 * @return true if read succeeded and CRC verified, false on any error.
 */
bool leaf_sensor_read(int16_t *temp, uint16_t *humidity);

/* raw ModBus response for debugging */
extern volatile uint8_t leaf_raw_rx[9];

#endif
