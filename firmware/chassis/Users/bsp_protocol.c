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

void Protocol_Init(void) {
    /*
     * CubeMX configures USART2 RX DMA in CIRCULAR mode, but IDLE-line
     * detection needs NORMAL mode so we can restart after each frame.
     * Override the CubeMX setting here.
     */
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);

    /* Start DMA reception with IDLE-line interrupt */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* Search for next 0xAA sync byte starting from 'start' */
static int16_t Find_Sync(const uint8_t* buf, uint16_t start, uint16_t len) {
    for (uint16_t i = start; i < len; i++) {
        if (buf[i] == 0xAA) return (int16_t)i;
    }
    return -1;
}

void Protocol_Process(void) {
    if (!rx_flag) return;
    rx_flag = 0;

#if PROTOCOL_DEBUG
    printf("[PROTO] rx_len=%d raw:", rx_len);
    for (int i = 0; i < rx_len && i < 32; i++) printf(" %02X", rx_buff[i]);
    printf("\r\n");
#endif

    uint16_t offset = 0;

    /*
     * Loop to handle multiple frames in one DMA buffer.
     * Minimum frame: header(2) + type(1) + len(1) + crc(2) + min 1 data byte = 7.
     */
    while (offset + 7 <= rx_len) {
        /* Sync to 0xAA 0x55 frame header */
        if (rx_buff[offset] != 0xAA || rx_buff[offset + 1] != 0x55) {
            int16_t next = Find_Sync(rx_buff, offset + 1, rx_len);
            if (next < 0) break;            /* No more sync bytes in buffer */
            offset = (uint16_t)next;
            if (offset + 1 >= rx_len) break;
            if (rx_buff[offset + 1] != 0x55) {
                offset++;
                continue;
            }
        }

        /* Need at least 7 bytes from sync position */
        if (offset + 7 > rx_len) break;

        uint8_t  type      = rx_buff[offset + 2];
        uint8_t  dlen      = rx_buff[offset + 3];     /* payload data length */
        uint16_t total_len = (uint16_t)dlen + 6;       /* header(2) + type(1) + len(1) + data + crc(2) */

        /* Incomplete frame — wait for more data */
        if (offset + total_len > rx_len) break;

        /* Verify CRC16-CCITT over header + type + len + data */
        uint16_t expected = crc16_ccitt(&rx_buff[offset], total_len - 2);
        uint16_t received = (uint16_t)(rx_buff[offset + total_len - 2] << 8)
                          | rx_buff[offset + total_len - 1];

        if (expected != received) {
#if PROTOCOL_DEBUG
            printf("[PROTO] CRC fail: calc=%04X recv=%04X\r\n", expected, received);
#endif
            offset++;   /* Step forward 1 byte to search for next 0xAA */
            continue;
        }

        /* ---- Dispatch by type ---- */
        if (type == TYPE_MOTION_CMD && dlen >= 4) {
            /* TYPE 0x81: Set wheel speeds (little-endian int16 pairs)
             * NOTE: Left/Right swapped because motor driver channels are
             * physically crossed on this hardware revision. */
            int16_t l_spd = (int16_t)(rx_buff[offset + 5] << 8 | rx_buff[offset + 4]);
            int16_t r_spd = (int16_t)(rx_buff[offset + 7] << 8 | rx_buff[offset + 6]);
            target_speed_left  = (float)r_spd;   /* swapped */
            target_speed_right = (float)l_spd;   /* swapped */
#if PROTOCOL_DEBUG
            printf("[PROTO] CMD_SPEED L=%d R=%d\r\n", l_spd, r_spd);
#endif
        }
#if PROTOCOL_DEBUG
        else {
            printf("[PROTO] Unknown TYPE=0x%02X len=%d\r\n", type, dlen);
        }
#endif

        offset += total_len;  /* Advance past this frame */
    }

    /* Clear UART error flags that may have accumulated */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    /* Ensure UART state is READY before restarting DMA (ISR already set it,
       but be defensive in case of error-recovery paths) */
    if (huart2.RxState != HAL_UART_STATE_READY) {
        huart2.RxState = HAL_UART_STATE_READY;
    }

    /* Restart DMA + IDLE reception for the next frame */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/**
 * @brief  Send accumulated encoder pulse counts to RDK X5 via USART2 (blocking).
 *         Frame: AA 55 03 08 <encL[4]> <encR[4]> <CRC16>  (14 bytes)
 *         Each encoder value is int32_t little-endian (4 bytes).
 *         Called from main loop at ~10 Hz.
 * @note   14 bytes @115200 ≈ 1.2 ms, negligible overhead at 10 Hz.
 */
void Protocol_Send_Telemetry(int32_t enc_left, int32_t enc_right)
{
    uint8_t frame[14];

    /*
     * NOTE: Left/Right swapped in telemetry because motor channels are
     * physically crossed. enc_left is from the "left" PID (which actually
     * drives the right motor), enc_right is from the "right" PID (left motor).
     * Swapping here gives correct L/R assignment to the RDK X5.
     */
    frame[0] = FRAME_HEADER_0;                   /* Header */
    frame[1] = FRAME_HEADER_1;
    frame[2] = TYPE_CHASSIS;                     /* TYPE: chassis telemetry */
    frame[3] = 0x08;                             /* LEN: 8 bytes payload (2× int32) */
    frame[4] = (uint8_t)(enc_right & 0xFF);       /* Right accum → Left slot [4:0] */
    frame[5] = (uint8_t)((enc_right >> 8) & 0xFF);
    frame[6] = (uint8_t)((enc_right >> 16) & 0xFF);
    frame[7] = (uint8_t)((enc_right >> 24) & 0xFF);
    frame[8] = (uint8_t)(enc_left  & 0xFF);      /* Left accum  → Right slot [8:0] */
    frame[9] = (uint8_t)((enc_left >> 8) & 0xFF);
    frame[10] = (uint8_t)((enc_left >> 16) & 0xFF);
    frame[11] = (uint8_t)((enc_left >> 24) & 0xFF);

    uint16_t crc = crc16_ccitt(frame, 12);
    frame[12] = (uint8_t)(crc >> 8);             /* CRC high byte */
    frame[13] = (uint8_t)(crc & 0xFF);           /* CRC low byte */

    HAL_UART_Transmit(&huart2, frame, 14, 10);  /* 10 ms timeout */
}
