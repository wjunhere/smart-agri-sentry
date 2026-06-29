/* pid.h */
#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target_val;   // target
    float actual_val;   // actual (measurement)

    float err;          // current error = target - actual

    // Positional PID state
    float integral;     // accumulated integral term (clamped)
    float integral_max; // integral windup clamp limit
    float integral_sep; // integral separation threshold (|err| < sep → I active)
    float last_actual;  // previous measurement (for derivative-on-measurement)

    float output;       // last computed output (for monitoring)

    // Adaptive friction compensation (feed-forward)
    float friction;             // estimated friction offset (PWM units)
    float friction_learn_rate;  // adaptation speed (0.001 = slow, 0.01 = fast)
    int   steady_count;         // consecutive ticks near steady-state
} PID_TypeDef;

void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd);
float PID_Calc(PID_TypeDef* pid, float target, float actual);
void PID_Reset(PID_TypeDef* pid);

#endif

