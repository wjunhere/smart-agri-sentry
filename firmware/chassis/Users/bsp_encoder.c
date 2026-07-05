/**
 * bsp_encoder.c
 * Wheel speed measurement via TIM2 (left) and TIM3 (right) in encoder mode.
 *
 * KEY CHANGE: TIM counters are NEVER reset — instead we track cumulative
 * encoder pulses and compute per-tick delta from counter differences.
 * This eliminates fractional-pulse loss that occurred with reset.
 *
 * NOTE: CubeMX generates GPIO_NOPULL for encoder input pins, but encoders
 * need pull-ups to avoid floating inputs. We override the CubeMX GPIO config
 * here with GPIO_PULLUP. If you change pin assignments in CubeMX, update here too.
 */
#include "bsp_encoder.h"

/* Cumulative encoder pulse counters — never reset, always grow.
 * int32_t range: ±2.1B pulses ≈ ±190 km of travel. */
static int32_t cumul_left  = 0;
static int32_t cumul_right = 0;

/* Last-read raw TIM counter values for delta computation.
 * The 16-bit counters wrap naturally; (int16_t)(now - last) handles it. */
static uint16_t last_cnt_left  = 0;
static uint16_t last_cnt_right = 0;
static uint8_t  first_read_left  = 1;  /* skip delta on very first read */
static uint8_t  first_read_right = 1;

/* Low-pass filtered speed (updated every 10 ms PID tick).
 * Retained for PID feedback — raw delta is too noisy at low speeds. */
static float filtered_speed_left  = 0.0f;
static float filtered_speed_right = 0.0f;

/* Raw per-tick delta (pulses/10ms), no filtering. Sent in telemetry. */
static int16_t raw_delta_left  = 0;
static int16_t raw_delta_right = 0;

/* ---- Adaptive outlier-rejection filter ----
 *
 * Encoder glitches produce physically-impossible speed jumps.
 * The adaptive filter compares each raw delta against a "credible
 * acceleration" band centred on the current filtered speed.  Readings
 * inside the band get the normal alpha; outliers get heavy attenuation
 * (alpha × 0.15) to suppress the glitch without adding phase lag.
 *
 * max_innovation scales with speed: more pulses → larger credible jump. */
static uint16_t outlier_cnt_left  = 0;
static uint16_t outlier_cnt_right = 0;

/* Compute the maximum credible innovation (pulses/tick) for a given speed.
 *   Base: 4 pulses — always allow small jitter.
 *   Dynamic: 30% of |filtered_speed| — higher speed → larger jumps are credible.
 *   Hard cap: 25 pulses — anything beyond this is almost certainly a glitch. */
static float max_credible_innovation(float filtered_speed) {
    float abs_spd = (filtered_speed >= 0.0f) ? filtered_speed : -filtered_speed;
    float dyn = 4.0f + abs_spd * 0.30f;
    if (dyn > 25.0f) dyn = 25.0f;
    return dyn;
}

/* Reset encoder state (call on STOP to avoid stale readings) */
void Encoder_Reset_Filters(void) {
    filtered_speed_left  = 0.0f;
    filtered_speed_right = 0.0f;
    raw_delta_left  = 0;
    raw_delta_right = 0;
    outlier_cnt_left  = 0;
    outlier_cnt_right = 0;
    /* NOTE: counters are NOT reset — cumulative tracking requires continuity.
     * last_cnt_left/right and cumul are also preserved. */
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
 *
 *         COUNTER IS NEVER RESET.  Delta is computed as (int16_t)(now-last),
 *         which automatically handles the 16-bit TIM counter wrap-around.
 *         The delta is accumulated into cumul_left and also stored as
 *         raw_delta_left for telemetry.
 *
 * @param  alpha  1st-order LPF coefficient for PID feedback.
 * @retval Filtered pulse count for PID loop (smooth).
 */
int32_t Encoder_Get_Left_Speed(float alpha) {
    uint16_t now_cnt = __HAL_TIM_GET_COUNTER(&htim2);

    /* Compute delta with wrap-around handling.
     * Cast to int16_t: 0x0001 - 0xFFFE = 3 (correct for 16-bit wrap). */
    int16_t delta = (int16_t)(now_cnt - last_cnt_left);
    last_cnt_left = now_cnt;

    /* Skip the very first delta — it's the counter value since boot, not 10ms. */
    if (first_read_left) {
        first_read_left = 0;
        delta = 0;
    }

    /* Negate: left encoder A/B phases are wired such that forward rotation
     * produces negative counts. Flipping sign matches right encoder. */
    int16_t corrected = -delta;

    /* Accumulate into never-reset cumulative counter */
    cumul_left += corrected;
    raw_delta_left = corrected;

    /* Clamp alpha */
    if (alpha < 0.05f) alpha = 0.05f;
    if (alpha > 0.95f) alpha = 0.95f;

    /* ---- Adaptive outlier-rejection LPF ----
     * If the new reading deviates from the current filtered value by more
     * than is physically credible, it's treated as a glitch: alpha is
     * reduced by 85% so the glitch barely affects the output.
     * Genuine speed changes (within the credible band) get full alpha. */
    {
        float innovation = (float)corrected - filtered_speed_left;
        float max_innov  = max_credible_innovation(filtered_speed_left);
        float eff_alpha  = alpha;

        if (innovation > max_innov || innovation < -max_innov) {
            eff_alpha = alpha * 0.15f;   /* heavy attenuation for outliers */
            outlier_cnt_left++;
        }
        filtered_speed_left += eff_alpha * innovation;
    }
    return (int32_t)filtered_speed_left;
}

/**
 * @brief  Read right wheel speed (pulses per 10 ms PID period).
 *
 *         Same non-resetting delta logic as left encoder.
 */
int32_t Encoder_Get_Right_Speed(float alpha) {
    uint16_t now_cnt = __HAL_TIM_GET_COUNTER(&htim3);

    /* Compute delta with wrap-around handling */
    int16_t delta = (int16_t)(now_cnt - last_cnt_right);
    last_cnt_right = now_cnt;

    /* Skip the very first delta */
    if (first_read_right) {
        first_read_right = 0;
        delta = 0;
    }

    /* Accumulate into never-reset cumulative counter */
    cumul_right += delta;
    raw_delta_right = delta;

    /* Clamp alpha */
    if (alpha < 0.05f) alpha = 0.05f;
    if (alpha > 0.95f) alpha = 0.95f;

    /* ---- Adaptive outlier-rejection LPF ---- */
    {
        float innovation = (float)delta - filtered_speed_right;
        float max_innov  = max_credible_innovation(filtered_speed_right);
        float eff_alpha  = alpha;

        if (innovation > max_innov || innovation < -max_innov) {
            eff_alpha = alpha * 0.15f;
            outlier_cnt_right++;
        }
        filtered_speed_right += eff_alpha * innovation;
    }
    return (int32_t)filtered_speed_right;
}

/* =========================================================================
 * Raw delta accessors — no filtering, no phase lag.
 * These are the per-tick (10ms) pulse counts for telemetry.
 * ========================================================================= */
int16_t Encoder_Get_Left_Raw_Delta(void) {
    return raw_delta_left;
}

int16_t Encoder_Get_Right_Raw_Delta(void) {
    return raw_delta_right;
}

/* =========================================================================
 * Cumulative accessors — never-reset total pulse count since boot.
 * Useful for odometry: distance_m = cumulative / PULSES_PER_METER.
 * ========================================================================= */
int32_t Encoder_Get_Cumulative_Left(void) {
    return cumul_left;
}

int32_t Encoder_Get_Cumulative_Right(void) {
    return cumul_right;
}

/* =========================================================================
 * Outlier counter accessors — diagnostic info for tuning.
 * High counts → encoder wiring noise or vibration at certain speeds.
 * ========================================================================= */
uint16_t Encoder_Get_Left_Outlier_Count(void) {
    return outlier_cnt_left;
}

uint16_t Encoder_Get_Right_Outlier_Count(void) {
    return outlier_cnt_right;
}
