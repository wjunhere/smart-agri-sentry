#ifndef __LORA_TX_H
#define __LORA_TX_H

#include <stdint.h>
#include "usart.h"

int lora_tx_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* ── Diagnostic globals (readable via OpenOCD) ── */
extern volatile uint8_t  g_lora_tx_buf[48];   /* last frame sent */
extern volatile uint16_t g_lora_tx_len;        /* last frame length */
extern volatile uint8_t  g_lora_tx_status;     /* HAL status of last TX */
extern volatile uint16_t g_lora_tx_count;      /* total frames sent */

#endif
