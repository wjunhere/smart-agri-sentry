/**
 * bsp_protocol.c
 * RDK X5 <-> STM32 communication protocol via USART2 DMA + IDLE line detection.
 *
 * Frame format: 0xAA 0x55 + TYPE(1B) + LEN(1B) + DATA(N bytes) + CRC16(2B)
 * CRC16 = CRC16-CCITT (polynomial 0x1021, init 0xFFFF) over TYPE+LEN+DATA
 *
 * TX: DMA (non-blocking) to avoid stalling the main loop.
 */
#include "bsp_protocol.h"
#include "pid.h"
#include "pid_autotune.h"
#include "bsp_encoder.h"
#include <string.h>
#include <stdio.h>

uint8_t rx_buff[RX_BUFF_SIZE];
volatile uint8_t  rx_flag = 0;
volatile uint16_t rx_len  = 0;

volatile float target_speed_left  = 0.0f;
volatile float target_speed_right = 0.0f;

/* DMA TX buffer and busy flag (non-blocking telemetry) */
static uint8_t tx_buf[32];
static volatile uint8_t tx_busy = 0;

/* CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF) */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void Protocol_Init(void) {
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* Search for 2-byte sync header 0xAA 0x55 */
static int16_t find_sync(const uint8_t *buf, uint16_t start, uint16_t len) {
    for (uint16_t i = start; i + 1 < len; i++) {
        if (buf[i] == FRAME_HEADER_0 && buf[i + 1] == FRAME_HEADER_1)
            return (int16_t)i;
    }
    return -1;
}

void Protocol_Process(void) {
    if (!rx_flag) return;
    rx_flag = 0;

    uint16_t offset = 0;
    while (offset + 6 <= rx_len) {  /* min frame: 2 header + 1 type + 1 len + 0 data + 2 crc = 6 */
        if (rx_buff[offset] != FRAME_HEADER_0 || rx_buff[offset + 1] != FRAME_HEADER_1) {
            int16_t next = find_sync(rx_buff, offset + 1, rx_len);
            if (next < 0) break;
            offset = (uint16_t)next;
        }

        if (offset + 6 > rx_len) break;

        uint8_t  type  = rx_buff[offset + 2];
        uint8_t  dlen  = rx_buff[offset + 3];
        uint16_t total = 4 + dlen + 2;  /* header(2) + type(1) + len(1) + data(dlen) + crc(2) */

        if (offset + total > rx_len) break;

        /* CRC16 over TYPE + LEN + DATA */
        uint16_t calc_crc = crc16_ccitt(&rx_buff[offset + 2], 2 + dlen);
        uint16_t rx_crc   = (uint16_t)((rx_buff[offset + total - 2] << 8) |
                                        rx_buff[offset + total - 1]);

        if (calc_crc != rx_crc) {
            offset++;
            continue;
        }

        /* TYPE_MOTION_CMD (0x81): 4-byte payload, left/right speed in mm/s int16 LE.
         * Channels are physically crossed on this hardware revision.
         * Convert mm/s → internal units (pulses/10ms) for PID:
         *   pulses/10ms = mm/s × ENCODER_PULSES_PER_METER / 100000  */
        if (type == TYPE_MOTION_CMD && dlen == 4) {
            int16_t left_mm_s  = (int16_t)(rx_buff[offset + 4] | (rx_buff[offset + 5] << 8));
            int16_t right_mm_s = (int16_t)(rx_buff[offset + 6] | (rx_buff[offset + 7] << 8));
            float scale = 11035.0f / 100000.0f;   /* ENCODER_PULSES_PER_METER / 100000 */

            /* Cross-swap: host-left → right motor, host-right → left motor */
            target_speed_left  = (float)right_mm_s * scale;
            target_speed_right = (float)left_mm_s  * scale;
        }

        /* TYPE_AUTOTUNE_CMD (0x82): 5-byte payload, trigger PID auto-tuning.
         *   [0-1] speed_mm_s  uint16 LE — test speed in mm/s
         *   [2]   conserv_x100 uint8     — conservativeness * 100 (50–200)
         *   [3]   zone         uint8     — PID zone to update (0–3), 0xFF=auto
         *   [4]   reserved     uint8     — future use (must be 0) */
        else if (type == TYPE_AUTOTUNE_CMD && dlen >= 5) {
            extern PID_AutoTuner autotuner;

            uint16_t speed_mm_s = (uint16_t)(rx_buff[offset + 4] |
                                   ((uint16_t)rx_buff[offset + 5] << 8));
            uint8_t cons_x100   = rx_buff[offset + 6];
            int8_t  zone_req    = (int8_t)rx_buff[offset + 7];

            if (speed_mm_s == 0) {
                /* Speed 0 = abort tuning */
                if (AT_Is_Active(&autotuner)) {
                    AT_Abort(&autotuner);
                }
            } else if (!AT_Is_Active(&autotuner)) {
                float speed_pulses = (float)speed_mm_s *
                                     (ENCODER_PULSES_PER_METER / 100000.0f);
                float conservativeness = (float)cons_x100 / 100.0f;
                if (conservativeness < 0.3f)  conservativeness = 1.0f;
                if (conservativeness > 2.0f)  conservativeness = 2.0f;

                /* Map speed to zone if zone_req == -1 (0xFF) */
                int zone;
                if (zone_req < 0) {
                    if (speed_pulses < 2.0f)       zone = PID_ZONE_STOP;
                    else if (speed_pulses < 15.0f) zone = PID_ZONE_LOW;
                    else if (speed_pulses < 60.0f) zone = PID_ZONE_MED;
                    else                           zone = PID_ZONE_HIGH;
                } else {
                    zone = zone_req;
                }

                /* Stop motors before tuning */
                target_speed_left  = 0.0f;
                target_speed_right = 0.0f;

                AT_Start(&autotuner, speed_pulses, zone, conservativeness);
            }
        }

        offset += total;
    }

    memset(rx_buff, 0, RX_BUFF_SIZE);

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

/**
 * @brief  Send chassis status frame to RDK X5 via USART2 DMA (non-blocking).
 *
 *         NEW 19-byte payload format (v2):
 *           AA 55 03 13 <left_mm_s[2]> <right_mm_s[2]> <battery_x100[2]>
 *                        <alarm[1]> <left_pulse[4]> <right_pulse[4]>
 *                        <timestamp_ms[4]> <CRC16[2]>
 *         All multi-byte fields little-endian. Total 25 bytes.
 *         Called from main loop at ~10 Hz.
 *         Skips silently if a previous DMA TX is still in progress.
 */
void Protocol_Send_Chassis_Status(int16_t left_speed_mm_s, int16_t right_speed_mm_s,
                                   int16_t battery_x100, uint8_t alarm_bits,
                                   int32_t left_pulse, int32_t right_pulse)
{
    if (tx_busy) return;  /* previous TX still in progress, skip this frame */

    tx_buf[0] = FRAME_HEADER_0;                      /* Header */
    tx_buf[1] = FRAME_HEADER_1;
    tx_buf[2] = TYPE_CHASSIS;                        /* TYPE: chassis status */
    tx_buf[3] = 0x13;                                /* LEN: 19 bytes payload */

    /* left_speed_mm_s  int16 LE — bytes 4-5 */
    tx_buf[4] = (uint8_t)(left_speed_mm_s & 0xFF);
    tx_buf[5] = (uint8_t)((left_speed_mm_s >> 8) & 0xFF);
    /* right_speed_mm_s int16 LE — bytes 6-7 */
    tx_buf[6] = (uint8_t)(right_speed_mm_s & 0xFF);
    tx_buf[7] = (uint8_t)((right_speed_mm_s >> 8) & 0xFF);
    /* battery_x100     int16 LE — bytes 8-9 */
    tx_buf[8] = (uint8_t)(battery_x100 & 0xFF);
    tx_buf[9] = (uint8_t)((battery_x100 >> 8) & 0xFF);
    /* alarm_bits       uint8   — byte 10 */
    tx_buf[10] = alarm_bits;

    /* left_pulse       int32 LE — bytes 11-14 */
    tx_buf[11] = (uint8_t)(left_pulse & 0xFF);
    tx_buf[12] = (uint8_t)((left_pulse >> 8) & 0xFF);
    tx_buf[13] = (uint8_t)((left_pulse >> 16) & 0xFF);
    tx_buf[14] = (uint8_t)((left_pulse >> 24) & 0xFF);
    /* right_pulse      int32 LE — bytes 15-18 */
    tx_buf[15] = (uint8_t)(right_pulse & 0xFF);
    tx_buf[16] = (uint8_t)((right_pulse >> 8) & 0xFF);
    tx_buf[17] = (uint8_t)((right_pulse >> 16) & 0xFF);
    tx_buf[18] = (uint8_t)((right_pulse >> 24) & 0xFF);
    /* timestamp_ms     uint32 LE — bytes 19-22 */
    uint32_t ts = HAL_GetTick();
    tx_buf[19] = (uint8_t)(ts & 0xFF);
    tx_buf[20] = (uint8_t)((ts >> 8) & 0xFF);
    tx_buf[21] = (uint8_t)((ts >> 16) & 0xFF);
    tx_buf[22] = (uint8_t)((ts >> 24) & 0xFF);

    /* CRC16-CCITT over TYPE + LEN + DATA (bytes 2..22, length 21) */
    uint16_t crc = crc16_ccitt(&tx_buf[2], 21);
    tx_buf[23] = (uint8_t)(crc >> 8);
    tx_buf[24] = (uint8_t)(crc & 0xFF);

    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, tx_buf, 25);
}

/**
 * @brief  HAL UART TX complete callback — clears the DMA TX busy flag.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        tx_busy = 0;
    }
}
