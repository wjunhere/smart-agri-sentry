#include "lora_tx.h"

extern UART_HandleTypeDef huart3;

int lora_tx_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, data, len, timeout_ms);
    return (status == HAL_OK) ? 0 : -1;
}
