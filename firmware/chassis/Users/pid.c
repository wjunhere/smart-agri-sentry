/**
 * pid.c — Positional PID with adaptive friction compensation.
 *
 *   u(k) = Kp*e(k) + Ki*Σe(k) + Kd*(actual derivative) + friction_ff
 *
 * Friction learning:
 *   At steady-state (|e| small for N consecutive ticks), the integral term
 *   primarily opposes friction.  We slowly bleed this into a feed-forward
 *   term so the integral doesn't have to "re-learn" friction after each
 *   start/stop cycle.
 *
 * Anti-windup:
 *   - i_term clamped to ±OUTPUT_MAX directly.
 *   - Back-calculation: when output saturates, undo integral accumulation.
 *   - Integral separation: I frozen when |e| >= integral_sep.
 */
#include "pid.h"
#include <stdlib.h>

#define OUTPUT_MAX         999.0f   /* PWM_ARR */
#define STEADY_ERR_THRESH    5.0f   /* |err| below this → "steady state" */
#define STEADY_COUNT_MIN    50      /* 0.5 sec @ 100 Hz before adapting friction */

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target_val  = 0.0f;
    pid->actual_val  = 0.0f;
    pid->err         = 0.0f;

    pid->integral     = 0.0f;
    pid->integral_max = 0.0f;
    pid->integral_sep = 500.0f;
    pid->last_actual  = 0.0f;

    pid->output = 0.0f;

    /* Friction compensation — starts at 0, learned on-line */
    pid->friction           = 0.0f;
    pid->friction_learn_rate = 0.005f;  /* slow, smooth adaptation */
    pid->steady_count       = 0;
}

/**
 * @brief  Positional PID + adaptive friction feed-forward.
 * @retval Absolute PWM value (clamped to ±OUTPUT_MAX).
 */
float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    pid->target_val = target;
    pid->actual_val = actual;
    pid->err        = target - actual;

    float abs_err = (pid->err >= 0.0f) ? pid->err : -pid->err;

    /* ---- Proportional ---- */
    float p_term = pid->Kp * pid->err;

    /* ---- Integral (separation + accumulation) ---- */
    if (abs_err < pid->integral_sep) {
        pid->integral += pid->err;
    }
    /* else: freeze — large transient, prevent windup */

    float i_term = pid->Ki * pid->integral;

    /* Clamp i_term directly */
    if (i_term >  OUTPUT_MAX) i_term =  OUTPUT_MAX;
    if (i_term < -OUTPUT_MAX) i_term = -OUTPUT_MAX;

    /* ---- Derivative on measurement ---- */
    float d_term = pid->Kd * (pid->last_actual - actual);

    /* ---- Feed-forward: direction * learned friction ---- */
    float ff_term = 0.0f;
    if (target > 0.5f)      ff_term =  pid->friction;
    else if (target < -0.5f) ff_term = -pid->friction;

    /* ---- Adaptive friction learning ---- */
    /*
     * When the system is at steady-state (small error for enough ticks),
     * p_term ≈ 0, d_term ≈ 0, so output ≈ i_term + ff_term.
     * We learn the total "bias" (i_term + ff_term) as the friction estimate,
     * so that eventually ff_term handles ALL friction and i_term → 0.
     */
    if (abs_err < STEADY_ERR_THRESH && (target > 0.5f || target < -0.5f)) {
        pid->steady_count++;
        if (pid->steady_count >= STEADY_COUNT_MIN) {
            float total_bias = i_term + ff_term;
            float observed = (target > 0.0f) ? total_bias : -total_bias;
            if (observed < 0.0f) observed = 0.0f;
            pid->friction += pid->friction_learn_rate * (observed - pid->friction);
        }
    } else {
        pid->steady_count = 0;   /* transient — reset counter */
    }

    /* ---- Sum and clamp ---- */
    float out = p_term + i_term + d_term + ff_term;
    if (out >  OUTPUT_MAX) out =  OUTPUT_MAX;
    if (out < -OUTPUT_MAX) out = -OUTPUT_MAX;

    /* Back-calculation anti-windup */
    if ((out >=  OUTPUT_MAX && pid->err > 0.0f) ||
        (out <= -OUTPUT_MAX && pid->err < 0.0f)) {
        if (abs_err < pid->integral_sep) {
            pid->integral -= pid->err;
        }
    }

    /* ---- Update state ---- */
    pid->last_actual = actual;
    pid->output = out;

    return out;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->err        = 0.0f;
    pid->integral   = 0.0f;
    pid->last_actual = 0.0f;
    pid->output     = 0.0f;
    pid->steady_count = 0;
    /* Note: friction is NOT reset — it's learned over time and persists */
}
