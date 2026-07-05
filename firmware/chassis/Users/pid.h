/* pid.h — Positional PID with gain scheduling, auto-tune support */
#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"

/* Speed zones for gain scheduling (based on |ramped_target| in pulses/10ms).
 * Encoder = 11035 pulses/m.  Each zone carries its own Kp/Ki/Kd,
 * integral separation threshold, PWM slew limit, and encoder filter alpha. */
#define PID_ZONE_STOP   0   /* |target| < 1.5 pulses  → < 0.14 m/s */
#define PID_ZONE_LOW    1   /* |target| < 5.0 pulses  → < 0.45 m/s */
#define PID_ZONE_MED    2   /* |target| < 15.0 pulses → < 1.36 m/s */
#define PID_ZONE_HIGH   3   /* |target| >= 15.0 pulses → ≥ 1.36 m/s */

/* One entry in the gain schedule table */
typedef struct {
    float     Kp;
    float     Ki;
    float     Kd;
    float     integral_sep;    /* I frozen when |err| >= this */
    uint16_t  slew_max;        /* max PWM change per 10 ms PID tick */
    float     encoder_alpha;   /* 1st-order LPF coefficient (0-1) */
} PID_GainEntry;

typedef struct {
    /* --- Active gains (loaded from gain table each tick) --- */
    float Kp;
    float Ki;
    float Kd;

    float target_val;   /* raw target (RDK command) */
    float actual_val;   /* measurement */
    float err;          /* error = ramped_target - actual */

    /* Positional PID state */
    float integral;
    float integral_max;
    float integral_sep; /* per-zone threshold, loaded from table */
    float last_actual;  /* for derivative-on-measurement */

    float output;

    /* Adaptive friction feed-forward */
    float friction;
    float friction_learn_rate;
    int   steady_count;

    /* --- Gain scheduling --- */
    int   zone;             /* current speed zone (PID_ZONE_*) */
    float ramped_target;    /* soft-ramped internal target */
    float prev_ramped;      /* previous ramped_target (for accel FF) */
    int   prev_zone;        /* previous zone for hysteresis */

    /* --- Acceleration feed-forward ---
     * Adds K_accel * (ramped_target - prev_ramped) to output.
     * Compensates for motor+chassis inertia during speed changes.
     * Set by auto-tuner; 0 = disabled. */
    float K_accel;

    /* --- Dead-zone compensation ---
     * When ramped_target transitions from |target| < 0.5 to > 0.5,
     * a temporary PWM boost is added to overcome static friction,
     * then decays linearly to 0 over dead_zone_ticks. */
    float dead_zone_pwm;    /* learned dead-zone PWM (from friction estimate) */
    int   dead_zone_ticks;  /* remaining decay ticks (0 = inactive) */
} PID_TypeDef;

/* Look up a gain entry by zone index (read-only).  Always valid (clamped 0..3). */
const PID_GainEntry* PID_Get_Gain_Entry(int zone);

/* Mutable access for auto-tuner.  Returns NULL if zone is out of range. */
PID_GainEntry* PID_Get_Gain_Entry_Mutable(int zone);

void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd);
float PID_Calc(PID_TypeDef* pid, float target, float actual);
void PID_Reset(PID_TypeDef* pid);

#endif
