/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c — Intelligent Agricultural Robot Car
  * @brief          : STM32F407ZGTx | RDK X5 Protocol | PID Motor Control
  * @note           : HSI 16MHz, USART1 9600 baud. Clock config disabled pending fix.
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
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_TypeDef pid_left, pid_right;
int32_t speed_left  = 0, speed_right = 0;
float   out_left    = 0, out_right    = 0;
float   battery_voltage = 0.0f;
uint32_t last_pid_tick       = 0;
uint32_t last_print_tick     = 0;
uint32_t last_heartbeat_tick = 0;
uint32_t last_iwdg_tick      = 0;
uint32_t last_telem_tick     = 0;

#define CHASSIS_ALARM_COMM_ERROR 0x04
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
 * To enable: uncomment the body below, but be aware that USART1 baud
 * rate mismatch issues may occur and require debugging.
 * ======================================================================== */
void SystemClock_Config(void)
{
    /*
     * DISABLED — see note above.
     * When enabled, this function attempts HSE+PLL → 168MHz,
     * falls back to HSI+PLL → 168MHz, then bare HSI 16MHz.
     * After clock success, USART1 must be re-initialized with new BRR.
     */
    (void)0; /* no-op */
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
    /* At HSI 16MHz: APB2=16MHz, TIM1_CLK=32MHz, PSC=16 → PWM=32M/(16*1000)=2kHz
       Override PSC to 3 → 32M/(3*1000)=8kHz */
    TIM1->PSC = 3;
    Motor_Init();
    printf(" Motor PWM     : OK (TIM1 CH1/CH2, %.0f Hz @ HSI)\r\n",
           32000000.0f / ((TIM1->PSC + 1) * 1000.0f));

    /* Encoder (TIM2 left PA5/PB3, TIM3 right PA6/PC7) */
    MX_TIM2_Init();
    MX_TIM3_Init();
    Encoder_Init();
    printf(" Encoder       : OK (TIM2 left, TIM3 right)\r\n");

    /* ADC (ADC1 CH7=PA7 battery voltage, DMA circular) */
    MX_ADC1_Init();
    MX_ADC2_Init();
    BSP_ADC_Init();
    printf(" ADC           : OK (ADC1 CH7 battery)\r\n");

    /* RDK X5 Protocol (USART2 DMA + IDLE) */
    Protocol_Init();
    printf(" RDK X5 Proto  : OK\r\n");

    /*
     * Debug_Init prints ~900 chars of info+help at 9600 baud (~900ms).
     * Do this BEFORE starting IWDG so the watchdog doesn't fire mid-printf.
     */
    Debug_Init();
    printf(" Debug Console : OK\r\n");

    /* PID init (fast, no heavy printf) */
    PID_Init(&pid_left,  3.0f, 1.0f, 0.05f);
    PID_Init(&pid_right, 3.0f, 1.0f, 0.05f);
    printf(" PID           : OK (Kp=3.0 Ki=1.0 Kd=0.05)\r\n");

    /* IWDG: start LAST, right before the main loop (~1s timeout) */
    MX_IWDG_Init();
    printf(" IWDG          : OK\r\n");

    printf("========================================\r\n");
    printf(" Type HELP for commands.\r\n\r\n");

    /* ====================================================================
     * MAIN LOOP
     *   1. Protocol_Process  — USART2 RDK X5 RX frames
     *   2. Debug_Process     — USART1 command parser
     *   3. PID @ 100 Hz      — encoder → PID → PWM
     *   4. Chassis status @ 10 Hz — USART2 TX project-standard frame to RDK X5
     *   5. Heartbeat @ 2 Hz  — PA8 LED
     *   6. Status @ 0.2 Hz   — printf summary (every 5 sec)
     *   7. IWDG @ 10 Hz      — watchdog refresh
     * ==================================================================== */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        Protocol_Process();
        Debug_Process();

        /* PID loop @ 100 Hz */
        if ((int32_t)(now - last_pid_tick) >= 10) {
            last_pid_tick += 10;

            speed_left  = Encoder_Get_Left_Speed();
            speed_right = Encoder_Get_Right_Speed();

            out_left  = PID_Calc(&pid_left,  target_speed_left,  (float)speed_left);
            out_right = PID_Calc(&pid_right, target_speed_right, (float)speed_right);

            Motor_Set_PWM((int16_t)out_left, (int16_t)out_right);

            battery_voltage = BSP_Get_Battery_Voltage();
        }

        /* Chassis status to RDK X5 @ 10 Hz — project-standard frame */
        if ((int32_t)(now - last_telem_tick) >= 100) {
            last_telem_tick += 100;
            int16_t left_speed_mm_s = (int16_t)(speed_left * 0.14137167f / 1560.0f * 100.0f * 1000.0f);
            int16_t right_speed_mm_s = (int16_t)(speed_right * 0.14137167f / 1560.0f * 100.0f * 1000.0f);
            int16_t battery_x100 = (int16_t)(battery_voltage * 100.0f);

            uint8_t alarm_bits = 0;
            if (Protocol_Get_CommErrorCount() > 5) {
                alarm_bits |= CHASSIS_ALARM_COMM_ERROR;
            }

            Protocol_Send_Chassis_Status(left_speed_mm_s,
                                         right_speed_mm_s,
                                         battery_x100,
                                         alarm_bits,
                                         Encoder_Get_Left_Accum(),
                                         Encoder_Get_Right_Accum(),
                                         HAL_GetTick());
        }

        /* Heartbeat LED @ 2 Hz */
        if ((int32_t)(now - last_heartbeat_tick) >= 500) {
            last_heartbeat_tick += 500;
            Diag_Heartbeat_Toggle();
        }

        /* Status print @ 0.2 Hz (every 5 seconds, reduce console noise) */
        if ((int32_t)(now - last_print_tick) >= 5000) {
            last_print_tick += 5000;
            printf("[%05lu] Tgt L:%.0f R:%.0f | Spd L:%d R:%d | Acc L:%d R:%d (%.2f %.2f m) | PWM L:%.0f R:%.0f | Bat:%.2fV\r\n",
                   (unsigned long)now,
                   target_speed_left, target_speed_right,
                   (int)speed_left, (int)speed_right,
                   (int)Encoder_Get_Left_Accum(), (int)Encoder_Get_Right_Accum(),
                   (double)Encoder_Get_Left_Distance_M(), (double)Encoder_Get_Right_Distance_M(),
                   out_left, out_right,
                   (double)battery_voltage);
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
