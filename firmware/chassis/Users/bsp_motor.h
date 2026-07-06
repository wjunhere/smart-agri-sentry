/* bsp_motor.h */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch2;

/* PWM auto-reload value (TIM1, 8 kHz @ HSI 16 MHz with PSC=3, ARR=999) */
#define PWM_ARR 999

void Motor_Init(void);

/* Set motor PWM with dynamic slew-rate limiting.
 *   slew_max = max PWM units change per 10 ms PID tick.
 *   Typical per-zone values: STOP=25, LOW=45, MED=80, HIGH=150. */
void Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm, uint16_t slew_max);
#endif
