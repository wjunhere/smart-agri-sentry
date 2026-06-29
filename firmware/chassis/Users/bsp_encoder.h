/* bsp_encoder.h */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Encoder_Init(void);
int32_t Encoder_Get_Left_Speed(void);
int32_t Encoder_Get_Right_Speed(void);
void Encoder_Reset_Filters(void);

/*
 * MG540 encoder: PPR=13, gear ratio=1:30, wheel Ø=4.5cm
 *   pulses_per_rev  = 13 × 4 × 30 = 1560
 *   wheel_circ      = π × 0.045  ≈ 0.1414 m
 *   pulses_per_meter = 1560 / 0.1414 ≈ 11035
 */
#define ENCODER_PULSES_PER_METER  11035.0f
#define ENCODER_WHEEL_CIRC_M      0.14137167f

/* Non-clearing accumulated pulse counters (for telemetry / odometry) */
int32_t Encoder_Get_Left_Accum(void);
int32_t Encoder_Get_Right_Accum(void);
float   Encoder_Get_Left_Distance_M(void);
float   Encoder_Get_Right_Distance_M(void);
#endif
