/* bsp_motor.h */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch2;

// PWM 自动重装值 (假设 TIM1 频率配置为 20kHz, ARR=999)
#define PWM_ARR 999 

void Motor_Init(void);
void Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm);
#endif
