/**
 * bsp_protocol.c
 * RDK X5 <-> STM32 communication protocol via USART2 DMA + IDLE line detection.
 *
 * Frame format: 0xAA 0x55 + TYPE(1B) + LEN(1B) + DATA(N bytes) + CRC16(2B)
 * CRC16 = CRC16-CCITT over all preceding bytes (polynomial 0x1021, init 0xFFFF).
 */
#include "bsp_protocol.h"
#include <string.h>
#include <stdio.h>

/* Set to 0 to disable per-packet debug printf (reduces USART1 traffic) */
#define PROTOCOL_DEBUG  0

uint8_t rx_buff[RX_BUFF_SIZE];
volatile uint8_t  rx_flag = 0;
volatile uint16_t rx_len  = 0;

volatile float target_speed_left  = 0.0f;
volatile float target_speed_right = 0.0f;

static uint8_t comm_error_count = 0;

/* CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF) */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t tx_buf[32];
static volatile uint8_t tx_busy = 0;

void Protocol_Init(void) {
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

static int16_t find_sync(const uint8_t *buf, uint16_t start, uint16_t len) {
    for (uint16_t i = start; i + 1 < len; i++) {
        if (buf[i] == FRAME_HEADER_0 && buf[i + 1] == FRAME_HEADER_1) {
            return (int16_t)i;
        }
    }
    return -1;
}

static void handle_motion_cmd(const uint8_t *payload) {
    int16_t left_mm_s  = (int16_t)(payload[0] | (payload[1] << 8));
    int16_t right_mm_s = (int16_t)(payload[2] | (payload[3] << 8));

    // Hardware channels are crossed: host left maps to STM32 right and vice versa
    target_speed_left  = (float)right_mm_s * 11035.0f / 100000.0f;
    target_speed_right = (float)left_mm_s  * 11035.0f / 100000.0f;
}

void Protocol_Process(void) {
    if (!rx_flag) return;
    rx_flag = 0;

    uint16_t offset = 0;
    while (offset + 6 <= rx_len) {
        if (rx_buff[offset] != FRAME_HEADER_0 || rx_buff[offset + 1] != FRAME_HEADER_1) {
            int16_t next = find_sync(rx_buff, offset + 1, rx_len);
            if (next < 0) break;
            offset = (uint16_t)next;
        }

        if (offset + 6 > rx_len) break;
        uint8_t cmd = rx_buff[offset + 2];
        uint8_t dlen = rx_buff[offset + 3];
        uint16_t total = 4 + dlen + 2;
        if (offset + total > rx_len) break;

        uint16_t calc_crc = crc16_ccitt(&rx_buff[offset + 2], 2 + dlen);
        uint16_t rx_crc = (uint16_t)(rx_buff[offset + total - 2] |
                                     (rx_buff[offset + total - 1] << 8));

        if (calc_crc != rx_crc) {
            comm_error_count++;
            offset++;
            continue;
        }

        if (cmd == TYPE_MOTION_CMD && dlen == 4) {
            handle_motion_cmd(&rx_buff[offset + 4]);
        }

        offset += total;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    if (huart2.RxState != HAL_UART_STATE_READY) {
        huart2.RxState = HAL_UART_STATE_READY;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

uint8_t Protocol_Get_CommErrorCount(void) {
    return comm_error_count;
}

void Protocol_Clear_CommErrorCount(void) {
    comm_error_count = 0;
}

/**
 * @brief  Send full chassis status frame to RDK X5 via USART2 DMA.
 *         Frame: AA 55 03 13 <left_speed[2]> <right_speed[2]> <battery[2]> <alarm[1]> <left_pulse[4]> <right_pulse[4]> <timestamp[4]> <CRC16>  (25 bytes)
 *         All multi-byte fields are little-endian.
 *         Called from main loop at ~10 Hz.
 * @return 0 if DMA started, 1 if previous TX still in progress.
 */
void Protocol_Send_Chassis_Status(int16_t left_speed_mm_s, int16_t right_speed_mm_s,
                                    int16_t battery_x100, uint8_t alarm_bits,
                                    int32_t left_pulse, int32_t right_pulse,
                                    uint32_t timestamp_ms)
{
    if (tx_busy) return;

    tx_buf[0] = FRAME_HEADER_0;                     /* Header */
    tx_buf[1] = FRAME_HEADER_1;
    tx_buf[2] = TYPE_CHASSIS;                       /* TYPE: chassis status */
    tx_buf[3] = 0x13;                               /* LEN: 19 bytes payload */

    /* left_speed_mm_s  int16  little-endian */
    tx_buf[4] = (uint8_t)(left_speed_mm_s & 0xFF);
    tx_buf[5] = (uint8_t)((left_speed_mm_s >> 8) & 0xFF);
    /* right_speed_mm_s int16  little-endian */
    tx_buf[6] = (uint8_t)(right_speed_mm_s & 0xFF);
    tx_buf[7] = (uint8_t)((right_speed_mm_s >> 8) & 0xFF);
    /* battery_x100     int16  little-endian */
    tx_buf[8] = (uint8_t)(battery_x100 & 0xFF);
    tx_buf[9] = (uint8_t)((battery_x100 >> 8) & 0xFF);
    /* alarm_bits       uint8 */
    tx_buf[10] = alarm_bits;
    /* left_pulse       int32  little-endian */
    tx_buf[11] = (uint8_t)(left_pulse & 0xFF);
    tx_buf[12] = (uint8_t)((left_pulse >> 8) & 0xFF);
    tx_buf[13] = (uint8_t)((left_pulse >> 16) & 0xFF);
    tx_buf[14] = (uint8_t)((left_pulse >> 24) & 0xFF);
    /* right_pulse      int32  little-endian */
    tx_buf[15] = (uint8_t)(right_pulse & 0xFF);
    tx_buf[16] = (uint8_t)((right_pulse >> 8) & 0xFF);
    tx_buf[17] = (uint8_t)((right_pulse >> 16) & 0xFF);
    tx_buf[18] = (uint8_t)((right_pulse >> 24) & 0xFF);
    /* timestamp_ms     uint32 little-endian */
    tx_buf[19] = (uint8_t)(timestamp_ms & 0xFF);
    tx_buf[20] = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    tx_buf[21] = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    tx_buf[22] = (uint8_t)((timestamp_ms >> 24) & 0xFF);

    /* CRC16-CCITT over TYPE + LEN + DATA (bytes 2..23, length 21) */
    uint16_t crc = crc16_ccitt(&tx_buf[2], 21);
    tx_buf[23] = (uint8_t)(crc >> 8);               /* CRC high byte */
    tx_buf[24] = (uint8_t)(crc & 0xFF);             /* CRC low byte */

    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, tx_buf, 25);
}

/**
 * @brief  HAL UART TX complete callback. Clears tx_busy when USART2 TX finishes.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        tx_busy = 0;
    }
}
