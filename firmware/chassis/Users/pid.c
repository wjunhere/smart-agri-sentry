/**
 * pid.c — Positional PID with gain scheduling, adaptive friction,
 *         acceleration feed-forward, and dead-zone compensation.
 *
 * Key design rules:
 * 1. integral_sep scales UP with speed zone (more room for I at high speeds)
 * 2. Friction learning threshold scales with target speed (15-20% of |target|)
 * 3. Kp stays moderate (2.0-2.5) across all zones for adequate P-term
 * 4. Kd increases at higher speeds for stronger oscillation damping
 * 5. Accel FF compensates for inertia during speed changes
 * 6. Dead-zone boost overcomes static friction when starting from stop
 */
#include "pid.h"
#include <stdlib.h>

#define OUTPUT_MAX          999.0f
#define STEADY_ERR_THRESH     5.0f   /* base threshold, scaled by zone */
#define STEADY_COUNT_MIN      30     /* 0.3 s @ 100 Hz (reduced for faster learning) */

/* =========================================================================
 * Gain schedule table — MUTABLE (auto-tuner writes to it at runtime).
 * Zone thresholds (pulses/10ms, ~11035 pulses/m):
 *   STOP  < 2.0    → < 0.018 m/s
 *   LOW   < 15.0   → < 0.136 m/s
 *   MED   < 60.0   → < 0.544 m/s
 *   HIGH  ≥ 60.0   → ≥ 0.544 m/s
 *
 * integral_sep is LARGER at higher speeds — error scales with target.
 * ========================================================================= */
static PID_GainEntry gain_table[4] = {
    /* { Kp,   Ki,   Kd,   i_sep, slew, alpha } */
    {  2.8f, 0.40f,0.18f,  10.0f,  25,  0.25f },   /* STOP — gentle, overcome static friction */
    {  2.0f, 0.25f,0.30f,  30.0f,  50,  0.35f },   /* LOW  — reduced Ki for accurate feedback */
    {  1.8f, 0.20f,0.35f,  60.0f, 100,  0.40f },   /* MED  — lower Kp, stronger Kd */
    {  1.5f, 0.15f,0.45f, 100.0f, 160,  0.50f },   /* HIGH — conservative Kp, fast damping */
};

/* Soft ramp rate (pulses/tick²). 3.0 is conservative — avoids integral windup. */
#define TARGET_RAMP_RATE   3.0f

/* Hysteresis: down-shift requires falling below threshold * HYST_DOWN */
#define HYST_DOWN          0.70f

/* Zone thresholds (pulses/10ms) */
#define ZONE_THR_LOW        0.5f   /* lowered from 2.0: prevent dead-zone at low cmd_vel (~4.5 mm/s) */
#define ZONE_THR_MED       15.0f
#define ZONE_THR_HIGH      60.0f

/* Default acceleration feed-forward gain.
 * Units: PWM / (pulse/tick²).  Set to 0 to disable.
 * A reasonable starting value — auto-tuner can override. */
#define DEFAULT_K_ACCEL     0.12f

/* Dead-zone compensation:
 *   DECAY_TICKS  = how long the boost lasts (200 ms = 20 ticks)
 *   LEARN_FRAC   = fraction of friction estimate used as dead-zone PWM */
#define DZ_DECAY_TICKS      20
#define DZ_LEARN_FRAC       0.60f

const PID_GainEntry* PID_Get_Gain_Entry(int zone)
{
    if (zone < PID_ZONE_STOP) zone = PID_ZONE_STOP;
    if (zone > PID_ZONE_HIGH) zone = PID_ZONE_HIGH;
    return &gain_table[zone];
}

PID_GainEntry* PID_Get_Gain_Entry_Mutable(int zone)
{
    if (zone < PID_ZONE_STOP || zone > PID_ZONE_HIGH) return NULL;
    return &gain_table[zone];
}

static int select_zone(float abs_target, int prev_zone)
{
    float lo = ZONE_THR_LOW;
    float md = ZONE_THR_MED;
    float hi = ZONE_THR_HIGH;

    if (prev_zone == PID_ZONE_HIGH)       hi *= HYST_DOWN;
    else if (prev_zone == PID_ZONE_MED)   md *= HYST_DOWN;
    else if (prev_zone == PID_ZONE_LOW)   lo *= HYST_DOWN;

    if (abs_target < lo) return PID_ZONE_STOP;
    if (abs_target < md) return PID_ZONE_LOW;
    if (abs_target < hi) return PID_ZONE_MED;
    return PID_ZONE_HIGH;
}

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
    pid->integral_sep = 100.0f;
    pid->last_actual  = 0.0f;
    pid->output = 0.0f;
    pid->friction           = 0.0f;
    pid->friction_learn_rate = 0.005f;
    pid->steady_count       = 0;
    pid->zone          = PID_ZONE_STOP;
    pid->ramped_target = 0.0f;
    pid->prev_ramped   = 0.0f;
    pid->prev_zone     = PID_ZONE_STOP;
    pid->K_accel       = DEFAULT_K_ACCEL;
    pid->dead_zone_pwm = 0.0f;
    pid->dead_zone_ticks = 0;
}

float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    /* ---- Soft ramp ---- */
    if (target > pid->ramped_target + TARGET_RAMP_RATE) {
        pid->ramped_target += TARGET_RAMP_RATE;
    } else if (target < pid->ramped_target - TARGET_RAMP_RATE) {
        pid->ramped_target -= TARGET_RAMP_RATE;
    } else {
        pid->ramped_target = target;
    }

    /* ---- Direction change → reset integral + restart ramp ---- */
    if ((target > 0.0f && pid->target_val < 0.0f) ||
        (target < 0.0f && pid->target_val > 0.0f)) {
        pid->integral     = 0.0f;
        pid->ramped_target = 0.0f;
    }
    /* ---- Stop → continuously decay integral toward zero.
     *       Uses exponential decay (×0.90 per tick) so the motor
     *       comes to a clean stop within ~1 second. ---- */
    if (target < 0.5f && target > -0.5f) {
        pid->integral     *= 0.90f;
        pid->ramped_target = 0.0f;
    }
    pid->target_val = target;
    pid->actual_val = actual;

    /* ---- Select zone (hysteresis on ramped target) ---- */
    float abs_ramp = (pid->ramped_target >= 0.0f) ? pid->ramped_target
                                                   : -pid->ramped_target;
    pid->zone = select_zone(abs_ramp, pid->prev_zone);
    pid->prev_zone = pid->zone;

    const PID_GainEntry *g = &gain_table[pid->zone];
    pid->Kp           = g->Kp;
    pid->Ki           = g->Ki;
    pid->Kd           = g->Kd;
    pid->integral_sep = g->integral_sep;

    /* ---- Error ---- */
    pid->err = pid->ramped_target - actual;
    float abs_err = (pid->err >= 0.0f) ? pid->err : -pid->err;

    /* ---- Proportional ---- */
    float p_term = pid->Kp * pid->err;

    /* ---- Integral (separation + accumulation) ---- */
    if (abs_err < pid->integral_sep) {
        pid->integral += pid->err;
    }
    float i_term = pid->Ki * pid->integral;
    if (i_term >  OUTPUT_MAX) i_term =  OUTPUT_MAX;
    if (i_term < -OUTPUT_MAX) i_term = -OUTPUT_MAX;

    /* ---- Derivative on measurement ---- */
    float d_term = pid->Kd * (pid->last_actual - actual);

    /* ---- Friction feed-forward ---- */
    float ff_term = 0.0f;
    if (pid->ramped_target > 0.5f)       ff_term =  pid->friction;
    else if (pid->ramped_target < -0.5f) ff_term = -pid->friction;

    /* ---- Acceleration feed-forward ----
     * Adds torque proportional to desired acceleration, compensating
     * for motor+chassis inertia.  Reduces tracking lag during speed changes. */
    float accel_ff = 0.0f;
    if (pid->K_accel > 0.001f) {
        float accel = pid->ramped_target - pid->prev_ramped;
        accel_ff = pid->K_accel * accel;
    }

    /* ---- Dead-zone compensation ----
     * When starting from stop, the motor needs extra PWM to overcome
     * static friction.  This boost kicks in on the first tick with
     * non-zero target and decays linearly to 0 over DZ_DECAY_TICKS. */
    float dz_ff = 0.0f;
    float prev_abs = (pid->prev_ramped >= 0.0f) ? pid->prev_ramped : -pid->prev_ramped;
    if (prev_abs < 0.5f && abs_ramp >= 0.5f) {
        /* Transition from stop → moving.  Start the dead-zone boost. */
        /* If we have a friction estimate, use a fraction of it.
         * Otherwise use a conservative default. */
        if (pid->friction > 5.0f) {
            pid->dead_zone_pwm = pid->friction * DZ_LEARN_FRAC;
        } else {
            pid->dead_zone_pwm = 30.0f;  /* default: 30 PWM (~3 % duty) */
        }
        pid->dead_zone_ticks = DZ_DECAY_TICKS;
    }

    if (pid->dead_zone_ticks > 0) {
        /* Linear decay: at tick N, fraction = N / DZ_DECAY_TICKS */
        float frac = (float)pid->dead_zone_ticks / (float)DZ_DECAY_TICKS;
        /* Apply in the direction of travel */
        float sign = (pid->ramped_target >= 0.0f) ? 1.0f : -1.0f;
        dz_ff = sign * pid->dead_zone_pwm * frac;
        pid->dead_zone_ticks--;
    }

    /* ---- Adaptive friction learning (threshold scales with speed) ---- */
    float steady_thresh = STEADY_ERR_THRESH;
    if (abs_ramp > 30.0f)       steady_thresh = abs_ramp * 0.15f;  /* 15% at high speed */
    else if (abs_ramp > 10.0f)  steady_thresh = abs_ramp * 0.25f;  /* 25% at medium speed */

    if (abs_err < steady_thresh &&
        (pid->ramped_target > 0.5f || pid->ramped_target < -0.5f)) {
        pid->steady_count++;
        if (pid->steady_count >= STEADY_COUNT_MIN) {
            float total_bias = i_term + ff_term;
            float observed = (pid->ramped_target > 0.0f) ? total_bias : -total_bias;
            if (observed < 0.0f) observed = 0.0f;
            /* Faster learning: use exponential moving average with α=0.01
             * so the system adapts to changing surfaces within ~3 seconds. */
            pid->friction += pid->friction_learn_rate * (observed - pid->friction);
        }
    } else {
        pid->steady_count = 0;
    }

    /* ---- Sum and clamp ---- */
    float out = p_term + i_term + d_term + ff_term + accel_ff + dz_ff;
    if (out >  OUTPUT_MAX) out =  OUTPUT_MAX;
    if (out < -OUTPUT_MAX) out = -OUTPUT_MAX;

    /* ---- Back-calculation anti-windup ---- */
    if ((out >=  OUTPUT_MAX && pid->err > 0.0f) ||
        (out <= -OUTPUT_MAX && pid->err < 0.0f)) {
        if (abs_err < pid->integral_sep) {
            pid->integral -= pid->err;
        }
    }

    pid->last_actual = actual;
    pid->prev_ramped = pid->ramped_target;  /* store for next tick's accel FF */
    pid->output = out;
    return out;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->err         = 0.0f;
    pid->integral    = 0.0f;
    pid->last_actual = 0.0f;
    pid->output      = 0.0f;
    pid->steady_count = 0;
    pid->ramped_target = 0.0f;
    pid->prev_ramped   = 0.0f;
    pid->zone         = PID_ZONE_STOP;
    pid->prev_zone    = PID_ZONE_STOP;
    pid->dead_zone_ticks = 0;
}
