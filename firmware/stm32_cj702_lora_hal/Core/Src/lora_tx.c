#include "lora_tx.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* ── Diagnostic globals (readable via OpenOCD) ── */
volatile uint8_t  g_lora_tx_buf[48] = {0};   /* last frame sent */
volatile uint16_t g_lora_tx_len     = 0;      /* last frame length */
volatile uint8_t  g_lora_tx_status  = 0;      /* HAL status of last TX */
volatile uint16_t g_lora_tx_count   = 0;      /* total frames sent */

int lora_tx_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0) {
        return -1;
    }

    /* Save frame for OpenOCD debugging */
    if (len <= sizeof(g_lora_tx_buf)) {
        memcpy((void *)g_lora_tx_buf, data, len);
    }
    g_lora_tx_len = len;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, data, len, timeout_ms);
    g_lora_tx_status = (uint8_t)status;
    g_lora_tx_count++;

    return (status == HAL_OK) ? 0 : -1;
}
