/**
 * bsp_motor.c — Motor PWM via TIM1 CH1 (PE9, left) / CH2 (PE11, right).
 *
 * Slew-rate limiting prevents mechanical shock when the PID issues large
 * corrections.  The limit is now dynamic — gentler at low speeds where
 * the encoder signal is noisy (few pulses/tick), more responsive at high
 * speeds where the signal is clean.
 */

#include "bsp_motor.h"
#include "gpio.h"

uint16_t pwm_dma_buf[2] = { 0, 0 };

/* Previous PWM outputs for slew-rate computation */
static int16_t prev_left_pwm  = 0;
static int16_t prev_right_pwm = 0;

void Motor_Init(void) {

    __HAL_TIM_MOE_ENABLE(&htim1);


    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&pwm_dma_buf[0], 1);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)&pwm_dma_buf[1], 1);


    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
}

/**
 * @brief  Set motor PWM with dynamic slew-rate limiting.
 * @param  slew_max  Maximum PWM change per PID tick (10 ms).
 *                   Clamped internally to [10, 999] for safety.
 */
void Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm, uint16_t slew_max) {

    /* Safety clamp */
    if (slew_max < 10)  slew_max = 10;
    if (slew_max > 999) slew_max = 999;

    /* ---- Slew-rate limiting ---- */
    int16_t delta;
    delta = left_pwm - prev_left_pwm;
    if (delta >  (int16_t)slew_max) left_pwm = prev_left_pwm + (int16_t)slew_max;
    if (delta < -(int16_t)slew_max) left_pwm = prev_left_pwm - (int16_t)slew_max;
    delta = right_pwm - prev_right_pwm;
    if (delta >  (int16_t)slew_max) right_pwm = prev_right_pwm + (int16_t)slew_max;
    if (delta < -(int16_t)slew_max) right_pwm = prev_right_pwm - (int16_t)slew_max;

    prev_left_pwm  = left_pwm;
    prev_right_pwm = right_pwm;

    /* ---- Direction control ---- */
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


    if (left_pwm > PWM_ARR) left_pwm = PWM_ARR;
    if (right_pwm > PWM_ARR) right_pwm = PWM_ARR;

    pwm_dma_buf[0] = (uint16_t)left_pwm;
    pwm_dma_buf[1] = (uint16_t)right_pwm;
}
