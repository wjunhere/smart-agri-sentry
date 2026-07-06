/**
 * bsp_debug.c
 * USART1 line-based command parser for standalone MCU debugging.
 *
 * Supported commands (case-insensitive):
 *   L<value>           Set left  wheel target speed (e.g. L100 or L-50)
 *   R<value>           Set right wheel target speed
 *   LR <left> <right>  Set both speeds at once (e.g. LR 100 200)
 *   STOP               Stop both wheels (target = 0)
 *   STATUS             Print current system status
 *   INFO               Print chip/firmware info
 *   TUNE <mm_s> [C]    Auto-tune PID at speed (mm/s). C=0.5–1.5 conservativeness.
 *   TUNE STOP          Abort auto-tuning.
 *   TEST <L> <R>       Send a test RDK X5 protocol frame on USART2
 *   SEND <hex>         Send raw hex bytes on USART2 (e.g. SEND AA010400000000AE)
 *   HELP or ?          Show this help
 *
 * The PID loop still runs — with no encoders connected feedback stays 0,
 * so PWM output saturates at the configured limit when target != 0.
 */
#include "bsp_debug.h"
#include "bsp_protocol.h"
#include "bsp_encoder.h"
#include "pid.h"
#include "pid_autotune.h"
#include "pid_disturbance.h"
#include "cross_track.h"
#include "bsp_encoder.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern volatile float target_speed_left;
extern volatile float target_speed_right;
extern PID_TypeDef pid_left, pid_right;
extern PID_AutoTuner autotuner;
extern DOB_Observer dob_left, dob_right;
extern float dob_ff_left, dob_ff_right;
extern CTS_Stabilizer cts;

#define DBG_LINE_MAX  64

static uint8_t  dbg_rx_char;          /* single-char RX buffer for HAL */
static char     dbg_line[DBG_LINE_MAX];
static uint8_t  dbg_idx = 0;
static uint8_t  dbg_line_ready = 0;

static void Cmd_STATUS(void);
static void Cmd_INFO(void);
static void Cmd_HELP(void);
static void Cmd_TUNE(const char *args);
static void Cmd_TEST(int l_spd, int r_spd);
static void Cmd_SEND(const char *hex_str);
static void USART2_Send(const uint8_t *data, uint16_t len);

/* ---- UART RX callback (called from HAL_UART_IRQHandler in ISR context) ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;

    char c = (char)dbg_rx_char;

    if (c == '\r' || c == '\n') {
        if (dbg_idx > 0) {
            dbg_line[dbg_idx] = '\0';
            dbg_line_ready = 1;
            dbg_idx = 0;
        }
    } else if (c == '\b' || c == 0x7F) {
        /* Backspace */
        if (dbg_idx > 0) dbg_idx--;
    } else if (dbg_idx < DBG_LINE_MAX - 1) {
        dbg_line[dbg_idx++] = c;
    }
    /* Re-arm RX interrupt for the next character */
    HAL_UART_Receive_IT(&huart1, &dbg_rx_char, 1);
}

/* ---- Initialization ---- */
void Debug_Init(void) {
    /* Kick off the first RX interrupt; subsequent ones re-arm in callback */
    HAL_UART_Receive_IT(&huart1, &dbg_rx_char, 1);
    printf("\033[2J\033[H");  /* Clear screen (VT100) */
    Cmd_INFO();
    Cmd_HELP();
}

/* ---- Main-loop call (non-blocking) ---- */
void Debug_Process(void) {
    if (!dbg_line_ready) return;

    /* Copy line to stack buffer immediately to avoid ISR race,
       THEN clear the flag so the ISR can start a new line. */
    char line_copy[DBG_LINE_MAX];
    strncpy(line_copy, dbg_line, DBG_LINE_MAX - 1);
    line_copy[DBG_LINE_MAX - 1] = '\0';
    dbg_line_ready = 0;

    char *line = line_copy;

    /* Trim leading spaces */
    while (*line == ' ') line++;

    /* Echo for visibility in terminal */
    printf("> %s\r\n", line);

    /* ---- Parse ---- */
    if      (strncmp(line, "STOP", 4) == 0 || strncmp(line, "stop", 4) == 0) {
        target_speed_left  = 0.0f;
        target_speed_right = 0.0f;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
        Encoder_Reset_Filters();
        CTS_Reset(&cts);
        printf("  [OK] Both motors stopped (PID+CTS reset)\r\n");
    }
    else if (strncmp(line, "LR ", 3) == 0 || strncmp(line, "lr ", 3) == 0) {
        int l = 0, r = 0;
        if (sscanf(line + 3, "%d %d", &l, &r) == 2) {
            target_speed_left  = (float)l;
            target_speed_right = (float)r;
            printf("  [OK] Target: L=%d  R=%d\r\n", l, r);
        } else {
            printf("  [ERR] Usage: LR <left_speed> <right_speed>\r\n");
        }
    }
    else if (strncmp(line, "TUNE", 4) == 0 || strncmp(line, "tune", 4) == 0) {
        Cmd_TUNE(line + 4);
    }
    else if (strncmp(line, "TEST ", 5) == 0 || strncmp(line, "test ", 5) == 0) {
        int l = 0, r = 0;
        if (sscanf(line + 5, "%d %d", &l, &r) == 2) {
            Cmd_TEST(l, r);
        } else {
            printf("  [ERR] Usage: TEST <left_speed> <right_speed>\r\n");
        }
    }
    else if (strncmp(line, "SEND ", 5) == 0 || strncmp(line, "send ", 5) == 0) {
        Cmd_SEND(line + 5);
    }
    else if (line[0] == 'L' || line[0] == 'l') {
        int spd = 0;
        if (sscanf(line + 1, "%d", &spd) == 1) {
            target_speed_left = (float)spd;
            printf("  [OK] Left target = %d\r\n", spd);
        } else {
            printf("  [ERR] Usage: L<value>  (e.g. L100 or L-50)\r\n");
        }
    }
    else if (line[0] == 'R' || line[0] == 'r') {
        int spd = 0;
        if (sscanf(line + 1, "%d", &spd) == 1) {
            target_speed_right = (float)spd;
            printf("  [OK] Right target = %d\r\n", spd);
        } else {
            printf("  [ERR] Usage: R<value>  (e.g. R200 or R-100)\r\n");
        }
    }
    else if (strncmp(line, "STATUS", 6) == 0 || strncmp(line, "status", 6) == 0) {
        Cmd_STATUS();
    }
    else if (strncmp(line, "INFO", 4) == 0 || strncmp(line, "info", 4) == 0) {
        Cmd_INFO();
    }
    else if (strncmp(line, "HELP", 4) == 0   || strncmp(line, "help", 4) == 0 ||
             strncmp(line, "?", 1)   == 0) {
        Cmd_HELP();
    }
    else if (strlen(line) > 0) {
        printf("  [ERR] Unknown command. Type HELP for list.\r\n");
    }
}

/* ---- USART2 blocking transmit helper ---- */
static void USART2_Send(const uint8_t *data, uint16_t len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

/* ---- Commands ---- */

/* TUNE: Relay auto-tuning for PID gains.
 * Usage:
 *   TUNE <speed_mm_s> [conservativeness]
 *     speed_mm_s      — test speed in mm/s (e.g. 300 = 0.3 m/s)
 *     conservativeness — 0.5=gentle, 1.0=standard ZN (default), 1.5=aggressive
 *   TUNE STOP         — abort in-progress tuning */
static void Cmd_TUNE(const char *args) {
    /* Skip leading spaces */
    while (*args == ' ') args++;

    if (strncmp(args, "STOP", 4) == 0 || strncmp(args, "stop", 4) == 0) {
        if (AT_Is_Active(&autotuner)) {
            AT_Abort(&autotuner);
            printf("  [OK] Auto-tuning aborted.\r\n");
        } else {
            printf("  [INFO] Auto-tuner is not running.\r\n");
        }
        return;
    }

    if (AT_Is_Active(&autotuner)) {
        printf("  [ERR] Auto-tuner already running (state=%s).  "
               "Use TUNE STOP first.\r\n",
               AT_State_Name(autotuner.state));
        return;
    }

    int speed_mm_s = 0;
    float conservativeness = 1.0f;
    int n = sscanf(args, "%d %f", &speed_mm_s, &conservativeness);

    if (n < 1 || speed_mm_s <= 0) {
        printf("  [ERR] Usage: TUNE <speed_mm_s> [conservativeness]\r\n");
        printf("        e.g. TUNE 300       - tune at 0.3 m/s (standard)\r\n");
        printf("        e.g. TUNE 600 0.5   - tune at 0.6 m/s (conservative)\r\n");
        printf("        e.g. TUNE 300 1.5   - tune at 0.3 m/s (aggressive)\r\n");
        printf("        e.g. TUNE STOP      - abort tuning\r\n");
        return;
    }

    /* Convert mm/s → pulses/10ms */
    float speed_pulses = (float)speed_mm_s * (ENCODER_PULSES_PER_METER / 100000.0f);

    /* Map speed to PID zone */
    int zone;
    if (speed_pulses < 2.0f)       zone = PID_ZONE_STOP;
    else if (speed_pulses < 15.0f) zone = PID_ZONE_LOW;
    else if (speed_pulses < 60.0f) zone = PID_ZONE_MED;
    else                           zone = PID_ZONE_HIGH;

    /* Ensure car is stopped before tuning */
    target_speed_left  = 0.0f;
    target_speed_right = 0.0f;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    Encoder_Reset_Filters();
    CTS_Reset(&cts);

    printf("  [TUNE] Starting auto-tune...\r\n");
    printf("         Speed: %d mm/s = %.1f pulses/10ms\r\n",
           speed_mm_s, (double)speed_pulses);
    printf("         Zone: %d (%s)\r\n", zone,
           zone == PID_ZONE_STOP ? "STOP" :
           zone == PID_ZONE_LOW  ? "LOW"  :
           zone == PID_ZONE_MED  ? "MED"  : "HIGH");
    printf("         Conservativeness: %.1f\r\n", (double)conservativeness);
    printf("         *** Ensure car is on the ground with room to move! ***\r\n");

    if (!AT_Start(&autotuner, speed_pulses, zone, conservativeness)) {
        printf("  [ERR] Failed to start auto-tuner (busy?).\r\n");
    }
}

/* TEST: Send a simulated RDK X5 speed-control frame on USART2
 * New protocol frame: AA 55 81 04 <left_L> <left_H> <right_L> <right_H> <CRC16_H> <CRC16_L>
 */
static void Cmd_TEST(int l_spd, int r_spd) {
    uint8_t frame[10];
    int16_t l = (int16_t)l_spd;
    int16_t r = (int16_t)r_spd;

    frame[0] = 0xAA;                         /* Header byte 0 */
    frame[1] = 0x55;                         /* Header byte 1 */
    frame[2] = 0x81;                         /* TYPE: motion command */
    frame[3] = 0x04;                         /* LEN: 4 bytes payload */
    frame[4] = (uint8_t)(l & 0xFF);
    frame[5] = (uint8_t)((l >> 8) & 0xFF);
    frame[6] = (uint8_t)(r & 0xFF);
    frame[7] = (uint8_t)((r >> 8) & 0xFF);

    /* CRC16-CCITT over TYPE + LEN + DATA (bytes 2..7, length 6) */
    uint16_t crc = crc16_ccitt(&frame[2], 6);
    frame[8] = (uint8_t)(crc >> 8);
    frame[9] = (uint8_t)(crc & 0xFF);

    USART2_Send(frame, 10);

    printf("  [TEST] Sent RDK X5 frame on USART2:\r\n");
    printf("         AA 55 81 04 %02X %02X %02X %02X %02X %02X\r\n",
           frame[4], frame[5], frame[6], frame[7], frame[8], frame[9]);
    printf("         Speed: L=%d R=%d\r\n", l, r);
}

/* SEND: Send raw hex bytes on USART2
 * Example: SEND AA010400000000AE
 */
static void Cmd_SEND(const char *hex_str) {
    uint8_t buf[128];
    uint16_t len = 0;
    char byte_str[3] = {0};

    /* Skip leading spaces */
    while (*hex_str == ' ') hex_str++;

    /* Parse hex string */
    const char *p = hex_str;
    while (*p && len < sizeof(buf)) {
        /* Skip spaces */
        while (*p == ' ') p++;
        if (!*p) break;

        byte_str[0] = *p++;
        if (!*p) break;
        byte_str[1] = *p++;

        char *endptr;
        long val = strtol(byte_str, &endptr, 16);
        if (endptr != byte_str + 2) {
            printf("  [ERR] Invalid hex byte: %s\r\n", byte_str);
            return;
        }
        buf[len++] = (uint8_t)val;
    }

    if (len == 0) {
        printf("  [ERR] No data to send. Usage: SEND <hex bytes>\r\n");
        return;
    }

    USART2_Send(buf, len);

    printf("  [SEND] %d bytes on USART2:", len);
    for (int i = 0; i < len && i < 32; i++) {
        printf(" %02X", buf[i]);
    }
    if (len > 32) printf(" ...");
    printf("\r\n");
}

static void Cmd_STATUS(void) {
    extern int32_t speed_left, speed_right;
    extern float out_left, out_right;
    extern float battery_voltage;
    extern PID_TypeDef pid_left, pid_right;
    extern PID_AutoTuner autotuner;

    printf("  ===== STATUS =====\r\n");
    printf("  Target:  L=%.0f  R=%.0f\r\n", target_speed_left, target_speed_right);
    printf("  Encoder: L=%d  R=%d  ", (int)speed_left, (int)speed_right);
    if (speed_left == 0 && speed_right == 0 &&
        (target_speed_left != 0 || target_speed_right != 0)) {
        printf("(NO ENCODER - PWM will saturate!)\r\n");
    } else {
        printf("\r\n");
    }
    printf("  PWM out: L=%.0f  R=%.0f  (ARR=999)\r\n", out_left, out_right);
    printf("  DOB est: L=%.0f  R=%.0f  (disturbance PWM)\r\n",
           dob_ff_left, dob_ff_right);
    printf("  Friction: L=%.0f  R=%.0f  (learned)\r\n",
           pid_left.friction, pid_right.friction);
    printf("  K_accel:  L=%.3f R=%.3f (accel FF)\r\n",
           pid_left.K_accel, pid_right.K_accel);
    printf("  Outliers: L=%d  R=%d  (encoder glitch counter)\r\n",
           (int)Encoder_Get_Left_Outlier_Count(),
           (int)Encoder_Get_Right_Outlier_Count());
    printf("  CTS:      yaw=%.1f bias=%.1f/%.1f  evt: Y=%d B=%d C=%d\r\n",
           (double)cts.yaw_integral,
           (double)cts.bias_left, (double)cts.bias_right,
           (int)cts.yaw_events, (int)cts.balance_events, (int)cts.consensus_events);
    printf("  Battery: %.2f V  (comp scale: %.2f)\r\n",
           battery_voltage,
           (double)(12.6f / (battery_voltage > 8.0f ? battery_voltage : 12.6f)));
    printf("  Uptime:  %lu ms\r\n", (unsigned long)HAL_GetTick());
    printf("  USART1:  %lu baud (this console)\r\n", (unsigned long)huart1.Init.BaudRate);
    printf("  USART2:  %lu baud DMA (RDK X5 protocol)\r\n", (unsigned long)huart2.Init.BaudRate);

    /* Auto-tuner status */
    if (AT_Is_Active(&autotuner)) {
        printf("  AutoTune: %s | tgt=%.1f p/tick  cycles=%d\r\n",
               AT_State_Name(autotuner.state),
               (double)autotuner.test_speed,
               autotuner.half_cycles / 2);
    } else if (autotuner.state == AT_DONE) {
        printf("  AutoTune: DONE  Kp=%.3f Ki=%.3f Kd=%.3f\r\n",
               (double)autotuner.tuned_Kp,
               (double)autotuner.tuned_Ki,
               (double)autotuner.tuned_Kd);
    } else if (autotuner.state == AT_ERROR) {
        printf("  AutoTune: ERROR - %s\r\n",
               autotuner.error_msg ? autotuner.error_msg : "unknown");
    } else {
        printf("  AutoTune: idle\r\n");
    }

    /* Zone gains */
    for (int z = 0; z < 4; z++) {
        const PID_GainEntry *g = PID_Get_Gain_Entry(z);
        printf("  Zone %d: Kp=%.3f Ki=%.3f Kd=%.3f i_sep=%.0f slew=%u alpha=%.2f\r\n",
               z, (double)g->Kp, (double)g->Ki, (double)g->Kd,
               (double)g->integral_sep, (unsigned)g->slew_max, (double)g->encoder_alpha);
    }

    printf("  ==================\r\n");
}

static void Cmd_INFO(void) {
    /* Determine clock source */
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t lat;
    HAL_RCC_GetClockConfig(&clk_cfg, &lat);

    const char *clk_src;
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk  = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    /* Check HSE status */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) {
        clk_src = "HSE (external crystal)";
    } else if (RCC->CFGR & RCC_CFGR_SWS_HSE) {
        clk_src = "HSE";
    } else {
        clk_src = "HSI (internal 16MHz - HSE failed or absent)";
    }

    printf("  ===== SYSTEM INFO =====\r\n");
    printf("  Chip:    STM32F407ZGTx (Cortex-M4)\r\n");
    printf("  Flash:   1024 KB\r\n");
    printf("  RAM:     192 KB (128+64 CCM)\r\n");
    printf("  Clock:   %s\r\n", clk_src);
    printf("  SYSCLK:  %lu MHz\r\n", (unsigned long)(sysclk / 1000000));
    printf("  HCLK:    %lu MHz\r\n", (unsigned long)(hclk / 1000000));
    printf("  PCLK1:   %lu MHz\r\n", (unsigned long)(pclk1 / 1000000));
    printf("  PCLK2:   %lu MHz\r\n", (unsigned long)(pclk2 / 1000000));
    printf("  FW Ver:  v1.3 (PID+DOB+CTS+AutoTune+BatComp)\r\n");
    printf("  =======================\r\n");
}

static void Cmd_HELP(void) {
    printf("  ===== STM32 Debug Console =====\r\n");
    printf("  Motor Control:\r\n");
    printf("    L<value>          set left  speed (e.g. L100)\r\n");
    printf("    R<value>          set right speed (e.g. R200)\r\n");
    printf("    LR <L> <R>        set both speeds\r\n");
    printf("    STOP              stop both motors\r\n");
    printf("  PID Auto-Tune:\r\n");
    printf("    TUNE <mm_s> [C]   auto-tune PID (e.g. TUNE 300)\r\n");
    printf("    TUNE STOP         abort tuning\r\n");
    printf("  Info:\r\n");
    printf("    STATUS            print voltage/speed/PWM/zone gains\r\n");
    printf("    INFO              print chip/firmware info\r\n");
    printf("  USART2 Test (RDK X5 protocol):\r\n");
    printf("    TEST <L> <R>      send speed frame on USART2\r\n");
    printf("    SEND <hex>        send raw bytes on USART2\r\n");
    printf("  Other:\r\n");
    printf("    HELP or ?         show this help\r\n");
    printf("  ===============================\r\n");
}
