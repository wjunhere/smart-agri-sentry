#ifndef __LORA_TX_H
#define __LORA_TX_H

#include <stdint.h>
#include "usart.h"

int lora_tx_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#endif
