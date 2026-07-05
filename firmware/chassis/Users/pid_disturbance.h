/**
 * pid_disturbance.h — Reduced-order disturbance observer (DOB).
 *
 * Principle:
 *   The motor obeys:   accel = motor_gain * (PWM - disturbance)
 *   where "disturbance" lumps together load torque, slope, grass, etc.
 *   The observer compares expected acceleration (from PWM model) against
 *   actual acceleration (from encoder delta), low-passes the error, and
 *   feeds it forward to cancel the disturbance.
 *
 * This is a game-changer for farmland: slopes, grass resistance, and
 * changing soil conditions are automatically compensated without waiting
 * for the PID integral term to catch up.
 *
 * Usage:
 *   DOB_Init(&dob, motor_gain, obs_gain, ff_gain);
 *   // Every PID tick:
 *   float dob_ff = DOB_Update(&dob, speed_pulses_per_tick, total_pwm);
 */
#ifndef __PID_DISTURBANCE_H
#define __PID_DISTURBANCE_H

/* ---- One disturbance observer per motor ---- */
typedef struct {
    /* --- Configuration --- */
    float motor_gain;       /* PWM -> acceleration gain (pulses/tick² per PWM unit).
                               Typical: 0.05–0.20.  Set by auto-tuner or use default. */
    float observer_gain;    /* Convergence rate (0.01–0.20).  Higher = faster tracking
                               but more noise.  0.05 = 200 ms time constant. */
    float ff_gain;          /* Feed-forward scaling (0.7–1.3).  1.0 = full cancellation. */

    /* --- State --- */
    float disturb_est;      /* Estimated disturbance, PWM-equivalent units [-999, 999] */
    float last_speed;       /* Previous speed for acceleration computation */
    float last_pwm;         /* Previous total PWM (for debugging/diagnostics) */
} DOB_Observer;

/* ---- API ---- */

/* One-time init.  Pass motor_gain=0 to use a conservative default.
 *   obs_gain: 0.03–0.10 typical (0.05 = good default)
 *   ff_gain:  0.8–1.2  (1.0 = full cancellation) */
void DOB_Init(DOB_Observer *dob, float motor_gain, float obs_gain, float ff_gain);

/* Run one PID tick (10 ms).
 *   speed: current filtered encoder speed (pulses/10ms)
 *   pwm:   total PWM output from PID (before DOB correction)
 * Returns: feed-forward PWM to ADD to the PID output (0..±999). */
float DOB_Update(DOB_Observer *dob, float speed, float pwm);

/* Reset internal state (call when motors stop). */
void DOB_Reset(DOB_Observer *dob);

/* Read back the current disturbance estimate for diagnostics. */
float DOB_Get_Disturbance(const DOB_Observer *dob);

#endif /* __PID_DISTURBANCE_H */
