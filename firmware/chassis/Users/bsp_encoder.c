/**
 * bsp_encoder.c
 * Wheel speed measurement via TIM2 (left) and TIM3 (right) in encoder mode.
 *
 * NOTE: CubeMX generates GPIO_NOPULL for encoder input pins, but encoders
 * need pull-ups to avoid floating inputs. We override the CubeMX GPIO config
 * here with GPIO_PULLUP. If you change pin assignments in CubeMX, update here too.
 */
#include "bsp_encoder.h"

/* Low-pass filtered speed (updated every 10 ms PID tick) */
static float filtered_speed_left  = 0.0f;
static float filtered_speed_right = 0.0f;

/* Non-clearing accumulated pulse counters (for telemetry / odometry).
 * These keep growing forever — never reset in normal operation. */
static int32_t accum_left  = 0;
static int32_t accum_right = 0;

/* Reset encoder filters (call on STOP to avoid transient readings).
 * NOTE: accum counters are intentionally NOT cleared — they are for
 * cumulative odometry and should persist across start/stop cycles. */
void Encoder_Reset_Filters(void) {
    filtered_speed_left  = 0.0f;
    filtered_speed_right = 0.0f;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Encoder_Init(void) {
    /*
     * Override CubeMX GPIO settings: switch from NOPULL to PULLUP.
     * Encoder open-collector/dry-contact outputs float without pull-ups.
     */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;        /* Override CubeMX NOPULL */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* TIM2_CH1 — PA5 (left encoder A) */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2_CH2 — PB3 (left encoder B) */
    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM3_CH1 — PA6 (right encoder A) */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM3_CH2 — PC7 (right encoder B) */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Start encoder timers, reset counter to zero */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief  Read left wheel speed (pulses per 10 ms PID period).
 * @retval Filtered pulse count, range approx -32767 ~ +32767.
 */
int32_t Encoder_Get_Left_Speed(void) {
    int16_t raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /*
     * Negate: left encoder A/B phases are wired such that forward rotation
     * produces negative counts. Flipping sign here matches right encoder.
     */
    int16_t corrected = -raw_cnt;

    /* Accumulate raw pulse count for telemetry (never cleared) */
    accum_left += (int32_t)corrected;

    /* 1st-order LPF: cutoff ~3.7 Hz at 100 Hz sample rate (alpha=0.3) */
    filtered_speed_left = 0.7f * filtered_speed_left + 0.3f * (float)corrected;
    return (int32_t)filtered_speed_left;
}

int32_t Encoder_Get_Right_Speed(void) {
    int16_t raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* Accumulate raw pulse count for telemetry (never cleared) */
    accum_right += (int32_t)raw_cnt;

    filtered_speed_right = 0.7f * filtered_speed_right + 0.3f * (float)raw_cnt;
    return (int32_t)filtered_speed_right;
}

/* Non-clearing accumulated pulse counters */
int32_t Encoder_Get_Left_Accum(void) {
    return accum_left;
}

int32_t Encoder_Get_Right_Accum(void) {
    return accum_right;
}

float Encoder_Get_Left_Distance_M(void) {
    return (float)accum_left / ENCODER_PULSES_PER_METER;
}

float Encoder_Get_Right_Distance_M(void) {
    return (float)accum_right / ENCODER_PULSES_PER_METER;
}
