/**
 * bsp_protocol.c
 * RDK X5 <-> STM32 communication protocol via USART2 DMA + IDLE line detection.
 *
 * Frame format: 0xAA 0x55 + TYPE(1B) + LEN(1B) + DATA(N bytes) + CRC16(2B)
 * CRC16 = CRC16-CCITT over all preceding bytes (polynomial 0x1021, init 0xFFFF).
 */
#include "bsp_protocol.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>

uint8_t rx_buff[RX_BUFF_SIZE];
volatile uint8_t  rx_flag = 0;
volatile uint16_t rx_len  = 0;

volatile float target_speed_left  = 0.0f;
volatile float target_speed_right = 0.0f;

volatile float g_right_wheel_trim      = 0.95f;   /* <1.0 slows right wheel */
volatile float g_right_motor_pwm_scale = 1.00f;   /* <1.0 reduces right PWM directly */

extern PID_TypeDef pid_left, pid_right;

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

/* If robot veers left when driving straight, the right wheel is too fast;
 * reduce g_right_wheel_trim (e.g. 0.95f) or g_right_motor_pwm_scale. */

/* Runtime-configurable parameter IDs (must match RDK side). */
#define PARAM_RIGHT_WHEEL_TRIM      1
#define PARAM_RIGHT_MOTOR_PWM_SCALE 2
#define PARAM_LEFT_KP               3
#define PARAM_LEFT_KI               4
#define PARAM_LEFT_KD               5
#define PARAM_RIGHT_KP              6
#define PARAM_RIGHT_KI              7
#define PARAM_RIGHT_KD              8

static uint32_t last_cmd_ms = 0;

static float bytes_to_float(const uint8_t *b)
{
    float f;
    memcpy(&f, b, sizeof(f));
    return f;
}

static void handle_motion_cmd(const uint8_t *payload) {
    int16_t left_mm_s  = (int16_t)(payload[0] | (payload[1] << 8));
    int16_t right_mm_s = (int16_t)(payload[2] | (payload[3] << 8));

    /* Motor driver channels are physically crossed on this board:
     * host-left connects to the right motor driver input and vice versa. */

    target_speed_left  = (float)right_mm_s * 11035.0f / 100000.0f;
    target_speed_right = (float)left_mm_s  * 11035.0f / 100000.0f * g_right_wheel_trim;

    last_cmd_ms = HAL_GetTick();
}

static void handle_config_cmd(const uint8_t *payload)
{
    uint8_t id = payload[0];
    float value = bytes_to_float(&payload[1]);

    switch (id) {
        case PARAM_RIGHT_WHEEL_TRIM:
            g_right_wheel_trim = value;
            break;
        case PARAM_RIGHT_MOTOR_PWM_SCALE:
            if (value >= 0.0f && value <= 1.5f) {
                g_right_motor_pwm_scale = value;
            }
            break;
        case PARAM_LEFT_KP:
            pid_left.Kp = value;
            break;
        case PARAM_LEFT_KI:
            pid_left.Ki = value;
            break;
        case PARAM_LEFT_KD:
            pid_left.Kd = value;
            break;
        case PARAM_RIGHT_KP:
            pid_right.Kp = value;
            break;
        case PARAM_RIGHT_KI:
            pid_right.Ki = value;
            break;
        case PARAM_RIGHT_KD:
            pid_right.Kd = value;
            break;
        default:
            return;
    }

    /* Reset PID integrals/derivatives to avoid jumps when gains change. */
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);

    printf("[CONFIG] id=%u value=%.4f\r\n", (unsigned)id, (double)value);
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
        uint16_t rx_crc = (uint16_t)((rx_buff[offset + total - 2] << 8) |
                                      rx_buff[offset + total - 1]);

        if (calc_crc != rx_crc) {
            comm_error_count++;
            offset++;
            continue;
        }

        if (cmd == TYPE_MOTION_CMD && dlen == 4) {
            handle_motion_cmd(&rx_buff[offset + 4]);
        }
        else if (cmd == TYPE_CONFIG_CMD && dlen == 5) {
            handle_config_cmd(&rx_buff[offset + 4]);
        }

        offset += total;
    }

    memset(rx_buff, 0, RX_BUFF_SIZE);

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    /* Workaround: HAL_UARTEx_ReceiveToIdle_DMA occasionally leaves RxState
     * busy after IDLE fires; force READY before restarting reception. */
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

void Protocol_Check_Command_Timeout(void) {
    if (HAL_GetTick() - last_cmd_ms > CMD_TIMEOUT_MS) {
        target_speed_left  = 0.0f;
        target_speed_right = 0.0f;
    }
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
    static uint8_t first_call = 1;
    static uint16_t call_cnt = 0;
    call_cnt++;

    if (first_call) {
        first_call = 0;
        printf("[PROTO] first call #%u tx_busy=%u gState=%u RxState=%u\r\n",
               call_cnt, tx_busy, huart2.gState, huart2.RxState);
        printf("[PROTO] USART2 CR1=0x%08lX CR3=0x%08lX SR=0x%08lX BRR=%lu\r\n",
               (unsigned long)huart2.Instance->CR1,
               (unsigned long)huart2.Instance->CR3,
               (unsigned long)huart2.Instance->SR,
               (unsigned long)huart2.Instance->BRR);
    }

    if (tx_busy) {
        if (call_cnt % 50 == 0)
            printf("[PROTO] call #%u: tx_busy still set, skipping\r\n", call_cnt);
        return;
    }

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
    HAL_StatusTypeDef rc = HAL_UART_Transmit_DMA(&huart2, tx_buf, 25);
    if (rc != HAL_OK) {
        tx_busy = 0;
        printf("[PROTO] TX_DMA error call #%u rc=%d gState=%u SR=0x%08lX\r\n",
               call_cnt, (int)rc, huart2.gState, (unsigned long)huart2.Instance->SR);
    } else if (call_cnt <= 3 || call_cnt % 50 == 0) {
        printf("[PROTO] TX_DMA call #%u rc=%d gState=%u SR=0x%08lX\r\n",
               call_cnt, (int)rc, huart2.gState, (unsigned long)huart2.Instance->SR);
    }
}

/**
 * @brief  HAL UART TX complete callback. Clears tx_busy when USART2 TX finishes.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint16_t txcnt = 0;
    if (huart == &huart2) {
        tx_busy = 0;
        txcnt++;
        if (txcnt <= 3)
            printf("[PROTO] TX DMA complete #%u SR=0x%08lX\r\n",
                   txcnt, (unsigned long)huart2.Instance->SR);
    }
}
