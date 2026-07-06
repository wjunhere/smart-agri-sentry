/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c — Intelligent Agricultural Robot Car
  * @brief          : STM32F407ZGTx | RDK X5 Protocol | PID + DOB + Auto-Tune
  * @note           : HSI 16MHz, USART1 9600 baud. Clock config disabled pending fix.
  *
  * Full feature set (v1.3):
  *   - Gain-scheduled PID (4 zones: STOP/LOW/MED/HIGH)
  *   - Relay auto-tuning (Ziegler-Nichols, trigger via USART1 or USART2)
  *   - Battery voltage compensation (PWM scales with Vref/Vbat)
  *   - Acceleration feed-forward (inertia compensation)
  *   - Dead-zone compensation (static friction boost)
  *   - Disturbance observer (load/slope/grass auto-cancellation)
  *   - Adaptive encoder filter (outlier rejection)
  *   - Cross-Track Stabilizer (yaw drift + load balance + consensus)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
#include "bsp_protocol.h"
#include "bsp_debug.h"
#include "bsp_diag.h"
#include "pid.h"
#include "pid_autotune.h"
#include "pid_disturbance.h"
#include "cross_track.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_TypeDef     pid_left, pid_right;
PID_AutoTuner   autotuner;
DOB_Observer    dob_left, dob_right;
CTS_Stabilizer  cts;

int32_t speed_left  = 0, speed_right = 0;
float   out_left    = 0, out_right    = 0;
float   dob_ff_left = 0, dob_ff_right = 0;
float   battery_voltage = 0.0f;
uint32_t last_pid_tick       = 0;
uint32_t last_print_tick     = 0;
uint32_t last_heartbeat_tick = 0;
uint32_t last_iwdg_tick      = 0;
uint32_t last_telem_tick     = 0;
uint32_t last_autotune_print = 0;

/* Raw delta accumulators for telemetry (sum over 100 ms = 10 PID ticks). */
static int32_t telem_acc_left   = 0;
static int32_t telem_acc_right  = 0;
static int32_t telem_tick_count = 0;

/* Battery voltage compensation reference (full-charge voltage). */
#define VBAT_REF        12.6f
#define VBAT_SCALE_MIN   0.85f
#define VBAT_SCALE_MAX   1.25f
/* USER CODE END PV */

void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * printf retargeting for both MicroLIB and ARM full C library.
 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* Full C library retarget (when MicroLIB is disabled) */
#if !defined(__MICROLIB)
#include <rt_misc.h>
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { while(1); }
int _write(int fd, const unsigned char *buf, unsigned int len)
{
    if (fd == 1) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
        return len;
    }
    return -1;
}
#endif

/* Boot marker via bare-metal USART1 */
static void BootMarker(char c)
{
    volatile uint32_t w;
    while (!(USART1->SR & USART_SR_TXE)) { __NOP(); }
    USART1->DR = (uint8_t)c;
    for (w = 0; w < 200000; w++) { __NOP(); }
}
/* USER CODE END 0 */

/* ========================================================================
 * SystemClock_Config — currently DISABLED.
 * MCU stays at HSI 16 MHz (known-good configuration).
 * ======================================================================== */
void SystemClock_Config(void)
{
    (void)0; /* no-op — stay at HSI 16 MHz */
}

/*
 * ====================== main ======================
 */
int main(void)
{
    /* ====================================================================
     * STEP 0: Bare-metal USART1 at 9600 (HSI 16MHz) for early output
     * ==================================================================== */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DSB();
    /* PA9 TX */
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODER9)  | GPIO_MODER_MODER9_1;
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << 4))       | (7U << 4);
    GPIOA->OSPEEDR |= (3U << 18);
    /* PA10 RX */
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODER10) | GPIO_MODER_MODER10_1;
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << 8))       | (7U << 8);
    GPIOA->OSPEEDR |= (3U << 20);
    /* 9600-8N1-TX+RX */
    USART1->BRR  = 16000000 / 9600;
    USART1->CR1  = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART1->CR2  = 0;
    USART1->CR3  = 0;

    BootMarker('0');   /* pre-HAL */
    HAL_Init();
    BootMarker('1');   /* post-HAL */

    /* Skip SystemClock_Config — stay at HSI 16 MHz */
    BootMarker('2');   /* clock skipped */

    MX_GPIO_Init();
    /* Restore PA9/PA10 (MX_GPIO_Init sets them to analog) */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
                 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFU << 4) | (0xFU << 8)))
                  | (7U << 4) | (7U << 8);
    BootMarker('3');   /* post-GPIO */

    MX_DMA_Init();
    BootMarker('4');   /* post-DMA */

    /* ====================================================================
     * Full USART1 init (huart1 for printf + debug RX)
     * ==================================================================== */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.gState            = HAL_UART_STATE_READY;
    huart1.RxState           = HAL_UART_STATE_READY;
    huart1.ErrorCode         = HAL_UART_ERROR_NONE;
    HAL_UART_MspInit(&huart1);
    /* Manual BRR+CR1 to guarantee 8N1 */
    USART1->CR1 = 0;
    USART1->BRR = 16000000 / 9600;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    BootMarker('5');   /* USART1 full ready */

    /* ====================================================================
     * Print banner
     * ==================================================================== */
    printf("\r\n========================================\r\n");
    printf("  Intelligent Agricultural Robot Car\r\n");
    printf("  STM32F407ZGTx | RDK X5 Protocol\r\n");
    printf("  FW v1.3 | PID+DOB+AutoTune+CTS+BatComp\r\n");
    printf("  SYSCLK: HSI 16 MHz | USART1: 9600 baud\r\n");
    printf("========================================\r\n");

    /* ====================================================================
     * Peripheral Init
     * ==================================================================== */

    /* Heartbeat LED (PA8) */
    Diag_Heartbeat_Init();
    printf(" PA8 Heartbeat : OK\r\n");

    /* USART2 (RDK X5 protocol DMA + IDLE) */
    MX_USART2_UART_Init();
    printf(" USART2 DMA    : OK\r\n");

    /* Motor PWM (TIM1, CH1=PE9 left, CH2=PE11 right) */
    MX_TIM1_Init();
    TIM1->PSC = 3;
    Motor_Init();
    printf(" Motor PWM     : OK (TIM1 CH1/CH2, %.0f Hz @ HSI)\r\n",
           32000000.0f / ((TIM1->PSC + 1) * 1000.0f));

    /* Encoder (TIM2 left PA5/PB3, TIM3 right PA6/PC7) */
    MX_TIM2_Init();
    MX_TIM3_Init();
    Encoder_Init();
    printf(" Encoder       : OK (TIM2 left, TIM3 right, adaptive filter)\r\n");

    /* ADC (ADC1 CH7=PA7 battery voltage, DMA circular) */
    MX_ADC1_Init();
    MX_ADC2_Init();
    BSP_ADC_Init();
    printf(" ADC           : OK (ADC1 CH7 battery)\r\n");

    /* RDK X5 Protocol (USART2 DMA + IDLE) */
    Protocol_Init();
    printf(" RDK X5 Proto  : OK\r\n");

    Debug_Init();
    printf(" Debug Console : OK\r\n");

    /* PID init */
    PID_Init(&pid_left,  2.5f, 0.5f, 0.2f);
    PID_Init(&pid_right, 2.5f, 0.5f, 0.2f);
    printf(" PID           : OK (gain-scheduled, 4 zones)\r\n");

    /* Auto-tuner init */
    AT_Init(&autotuner);
    printf(" Auto-Tuner    : OK (relay method, Z-N rules)\r\n");

    /* Disturbance observer init — one per motor.
     * motor_gain=0 → use default (0.08).  Auto-tuner updates this after tuning. */
    DOB_Init(&dob_left,  0.0f, 0.05f, 0.90f);
    DOB_Init(&dob_right, 0.0f, 0.05f, 0.90f);
    printf(" DOB           : OK (left+right, g=0.05 ff=0.90)\r\n");

    /* Cross-track stabiliser for uneven farmland terrain */
    CTS_Init(&cts);
    printf(" Cross-Track   : OK (yaw+balance+consensus)\r\n");

    /* IWDG: start LAST, right before the main loop (~1s timeout) */
    MX_IWDG_Init();
    printf(" IWDG          : OK\r\n");

    printf("========================================\r\n");
    printf(" Type HELP for commands.\r\n\r\n");

    /* ====================================================================
     * MAIN LOOP — complete signal flow (100 Hz):
     *
     *   Encoder → Adaptive Filter → speed
     *     ↓
     *   CTS_Update(target, speed, DOB, PWM) → corrected targets
     *     ↓
     *   PID_Calc(corrected_target, speed) → pid_out
     *     ↓                                ↓
     *   DOB_Update(speed, pid_out) → dob_ff
     *     ↓
     *   total = pid_out + dob_ff
     *     ↓
     *   Battery compensaton: total *= Vref/Vbat
     *     ↓
     *   Motor_Set_PWM(total)
     *
     * When auto-tuner is active, the PID+DOB+CTS path is bypassed and the
     * tuner drives the motors directly via relay control.
     * ==================================================================== */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        Protocol_Process();
        Debug_Process();

        /* PID loop @ 100 Hz */
        if ((int32_t)(now - last_pid_tick) >= 10) {
            last_pid_tick = now;

            /* Encoder alpha from previous tick's zone */
            int prev_zone = (pid_left.zone > pid_right.zone) ? pid_left.zone : pid_right.zone;
            float enc_alpha = PID_Get_Gain_Entry(prev_zone)->encoder_alpha;

            speed_left  = Encoder_Get_Left_Speed(enc_alpha);
            speed_right = Encoder_Get_Right_Speed(enc_alpha);

            /* Accumulate raw deltas for telemetry */
            telem_acc_left  += Encoder_Get_Left_Raw_Delta();
            telem_acc_right += Encoder_Get_Right_Raw_Delta();
            telem_tick_count++;

            /* ---- Auto-tuner or PID+DOB ---- */
            if (AT_Is_Active(&autotuner)) {
                /* Auto-tuner takes over both motors. */
                float avg_speed = ((float)speed_left + (float)speed_right) * 0.5f;
                AT_Tick(&autotuner, avg_speed);

                Motor_Set_PWM(autotuner.pwm_left, autotuner.pwm_right, 999);

                /* Decay PID state for smooth handover */
                pid_left.integral  *= 0.90f;
                pid_right.integral *= 0.90f;
                pid_left.ramped_target  = 0.0f;
                pid_right.ramped_target = 0.0f;
                pid_left.prev_ramped    = 0.0f;
                pid_right.prev_ramped   = 0.0f;
                pid_left.dead_zone_ticks  = 0;
                pid_right.dead_zone_ticks = 0;

                /* Reset DOB and CTS during tuning */
                DOB_Reset(&dob_left);
                DOB_Reset(&dob_right);
                CTS_Reset(&cts);

                out_left  = (float)autotuner.pwm_left;
                out_right = (float)autotuner.pwm_right;
                dob_ff_left = dob_ff_right = 0.0f;

                /* Auto-tuner status @ 2 Hz */
                if ((int32_t)(now - last_autotune_print) >= 500) {
                    last_autotune_print = now;
                    printf("[AUTOTUNE] %s | tgt=%.1f spd=%.1f pwm=%d\r\n",
                           AT_State_Name(autotuner.state),
                           (double)autotuner.test_speed,
                           (double)avg_speed,
                           (int)autotuner.pwm_left);
                }

            } else {
                /* ---- Cross-Track Stabilizer ----
                 * Modifies targets to compensate for uneven terrain:
                 *   - Yaw drift: cancels cumulative L-R speed asymmetry
                 *   - Load balance: prevents free-spinning on unloaded track
                 *   - Consensus: catches bounce artifacts on one track
                 * Returns corrected targets that PID_Calc will track. */
                float cts_tgt_l = target_speed_left;
                float cts_tgt_r = target_speed_right;
                CTS_Update(&cts, (float)speed_left, (float)speed_right,
                           dob_ff_left, dob_ff_right,
                           &cts_tgt_l, &cts_tgt_r,
                           out_left, out_right);

                /* ---- Normal PID control ---- */
                out_left  = PID_Calc(&pid_left,  cts_tgt_l, (float)speed_left);
                out_right = PID_Calc(&pid_right, cts_tgt_r, (float)speed_right);

                /* ---- Disturbance observer ----
                 * Estimates external load (slope, grass, soil) from the
                 * discrepancy between expected and actual acceleration.
                 * Feeds forward to cancel the disturbance BEFORE the PID
                 * integral has to wind up.  Critical for farmland. */
                dob_ff_left  = DOB_Update(&dob_left,  (float)speed_left,  out_left);
                dob_ff_right = DOB_Update(&dob_right, (float)speed_right, out_right);

                out_left  += dob_ff_left;
                out_right += dob_ff_right;

                /* ---- Battery voltage compensation ----
                 * Motor torque ∝ PWM × Vbat.  Scale PWM to maintain
                 * consistent torque as battery discharges. */
                float vbat = battery_voltage;
                if (vbat < 8.0f)  vbat = 8.0f;
                if (vbat > 15.0f) vbat = 15.0f;
                float vbat_scale = VBAT_REF / vbat;
                if (vbat_scale < VBAT_SCALE_MIN) vbat_scale = VBAT_SCALE_MIN;
                if (vbat_scale > VBAT_SCALE_MAX) vbat_scale = VBAT_SCALE_MAX;

                out_left  *= vbat_scale;
                out_right *= vbat_scale;

                /* Motor slew from current zone */
                int cur_zone = (pid_left.zone > pid_right.zone) ? pid_left.zone : pid_right.zone;
                uint16_t slew = PID_Get_Gain_Entry(cur_zone)->slew_max;
                Motor_Set_PWM((int16_t)out_left, (int16_t)out_right, slew);
            }

            battery_voltage = BSP_Get_Battery_Voltage();
        }

        /* Chassis status to RDK X5 @ 10 Hz (19-byte v2 frame). */
        if ((int32_t)(now - last_telem_tick) >= 100) {
            last_telem_tick += 100;

            float mm_s_factor = 100000.0f / ENCODER_PULSES_PER_METER;
            int16_t left_mm_s  = (int16_t)((float)speed_left  * mm_s_factor);
            int16_t right_mm_s = (int16_t)((float)speed_right * mm_s_factor);

            int32_t left_cumul  = Encoder_Get_Cumulative_Left();
            int32_t right_cumul = Encoder_Get_Cumulative_Right();

            telem_acc_left  = 0;
            telem_acc_right = 0;
            telem_tick_count = 0;

            int16_t bat_x100 = (int16_t)(battery_voltage * 100.0f);
            Protocol_Send_Chassis_Status(left_mm_s, right_mm_s, bat_x100, 0,
                                         left_cumul, right_cumul);
        }

        /* Heartbeat LED @ 2 Hz */
        if ((int32_t)(now - last_heartbeat_tick) >= 500) {
            last_heartbeat_tick += 500;
            Diag_Heartbeat_Toggle();
        }

        /* Status print @ 0.2 Hz (5s), only when stopped. */
        if ((int32_t)(now - last_print_tick) >= 5000) {
            last_print_tick += 5000;
            if (target_speed_left == 0.0f && target_speed_right == 0.0f
                && !AT_Is_Active(&autotuner)) {
                static const char *zone_names[4] = {"STOP","LOW ","MED ","HIGH"};
                printf("[%05lu] Z:%s | Enc L:%d R:%d | "
                       "PWM L:%.0f R:%.0f | DOB L:%.0f R:%.0f | "
                       "Fric L:%.0f R:%.0f | OutL: %d/%d | "
                       "CTS Y:%.0f B:%d/%d C:%d | Bat:%.2fV\r\n",
                       (unsigned long)now,
                       zone_names[pid_left.zone],
                       (int)speed_left, (int)speed_right,
                       out_left, out_right,
                       dob_ff_left, dob_ff_right,
                       (double)pid_left.friction, (double)pid_right.friction,
                       (int)Encoder_Get_Left_Outlier_Count(),
                       (int)Encoder_Get_Right_Outlier_Count(),
                       (double)cts.yaw_integral,
                       (int)cts.yaw_events, (int)cts.balance_events,
                       (int)cts.consensus_events,
                       (double)battery_voltage);
            }
        }

        /* IWDG refresh @ 10 Hz */
        if ((int32_t)(now - last_iwdg_tick) >= 100) {
            last_iwdg_tick += 100;
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

/* ==================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { __NOP(); }
}
/* USER CODE END 4 */
