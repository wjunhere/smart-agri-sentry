
#include "bsp_motor.h"
#include "bsp_protocol.h"
#include "gpio.h"

uint16_t pwm_dma_buf[2] = { 0, 0 };

void Motor_Init(void) {

    __HAL_TIM_MOE_ENABLE(&htim1);


    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&pwm_dma_buf[0], 1);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)&pwm_dma_buf[1], 1);


    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
}

void Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm) {

    if (left_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
    }
    else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        left_pwm = -left_pwm;
    }


    if (right_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
        right_pwm = -right_pwm;
    }

    /* Apply per-motor PWM scaling (runtime tunable). */
    right_pwm = (int16_t)((float)right_pwm * g_right_motor_pwm_scale + 0.5f);


    if (left_pwm > PWM_ARR) left_pwm = PWM_ARR;
    if (right_pwm > PWM_ARR) right_pwm = PWM_ARR;

    pwm_dma_buf[0] = (uint16_t)left_pwm;
    pwm_dma_buf[1] = (uint16_t)right_pwm;
}