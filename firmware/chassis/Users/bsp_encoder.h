/* bsp_encoder.h */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Encoder_Init(void);

/* Read wheel speeds.  alpha = 1st-order LPF coefficient (0.05–0.95).
 *   alpha ~0.2  → heavy filtering, ~2.7 Hz cutoff (low-speed)
 *   alpha ~0.55 → light filtering, ~8.8 Hz cutoff (high-speed)
 * The caller should pass the encoder_alpha from the current PID_GainEntry.
 * Returns FILTERED speed — use this for PID feedback. */
int32_t Encoder_Get_Left_Speed(float alpha);
int32_t Encoder_Get_Right_Speed(float alpha);

/* Raw per-tick delta (pulses/10ms), NO filtering, NO phase lag.
 * Use these for telemetry — the RDK gets honest instantaneous speed. */
int16_t Encoder_Get_Left_Raw_Delta(void);
int16_t Encoder_Get_Right_Raw_Delta(void);

/* Cumulative pulse counters — never reset since boot.
 * For odometry: distance_m = cumulative / ENCODER_PULSES_PER_METER. */
int32_t Encoder_Get_Cumulative_Left(void);
int32_t Encoder_Get_Cumulative_Right(void);

void Encoder_Reset_Filters(void);

/* Reset cumulative pulse counters to zero (for re-localisation / new mission).
 * Also resets last-read counters and first-read flags so the next delta
 * is zero (not the accumulated counter value since boot). */
void Encoder_Reset_Cumulative(void);

/* Adaptive filter outlier counters (for diagnostics).
 * Incremented each time a raw delta exceeds the physically-plausible range
 * and gets extra attenuation.  Resettable via Encoder_Reset_Filters(). */
uint16_t Encoder_Get_Left_Outlier_Count(void);
uint16_t Encoder_Get_Right_Outlier_Count(void);

/* MG540 encoder: PPR=13, gear=1:30, wheel dia=4.5cm → 11035 pulses/m */
#define ENCODER_PULSES_PER_METER  11035.0f
#endif
