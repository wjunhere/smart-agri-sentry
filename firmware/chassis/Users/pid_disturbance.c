/**
 * pid_disturbance.c — Reduced-order disturbance observer implementation.
 *
 * Theory of operation:
 *
 *   The motor + load dynamics are approximately:
 *     dω/dt ≈ motor_gain * (u - d)
 *
 *   where:
 *     ω   = wheel speed (pulses / PID tick)
 *     u   = PWM command (0..999)
 *     d   = disturbance (PWM-equivalent): load torque, slope, grass, friction
 *
 *   The observer predicts acceleration from the PWM command:
 *     a_pred[k] = motor_gain * u[k-1]
 *
 *   and compares it against the measured acceleration:
 *     a_meas[k] = ω[k] - ω[k-1]
 *
 *   The discrepancy is attributed to disturbance:
 *     innovation[k] = a_meas[k] - a_pred[k]
 *
 *   which is then low-pass filtered:
 *     d_hat[k] = d_hat[k-1] + observer_gain * (innovation[k] - d_hat[k-1])
 *
 *   The feed-forward cancels the estimated disturbance:
 *     u_ff[k] = ff_gain * d_hat[k]
 *
 * Key design choices:
 *   - 1st-order filter (not full Kalman) → ~50 bytes RAM, ~40 cycles/tick
 *   - Works even without accurate motor parameters (observer adapts)
 *   - Automatically handles changing load (grass → soil → slope)
 */
#include "pid_disturbance.h"
#include <string.h>

/* ---- Safe defaults (overridden if motor_gain is set by auto-tuner) ---- */

/* Default motor gain: ~0.08 pulses/tick² per PWM unit.
 * This was estimated from:
 *   - DC gain K_dc ≈ 30 pulses/tick / 300 PWM = 0.1 pulses/tick/PWM
 *   - Time constant τ ≈ 25 ticks
 *   - motor_gain = K_dc / τ ≈ 0.004...  but empirically 0.06–0.10 works better
 *     because the observer corrects its own model error. */
#define DEFAULT_MOTOR_GAIN      0.08f

/* Default observer gain: τ_obs ≈ 1/g ≈ 20 ticks (200 ms).
 * Fast enough to catch changing loads, slow enough to reject encoder noise. */
#define DEFAULT_OBSERVER_GAIN   0.05f

/* Feed-forward scaling: slightly under 1.0 to avoid oscillation. */
#define DEFAULT_FF_GAIN         0.90f

/* Clamp the disturbance estimate to prevent windup. */
#define DOB_CLAMP               500.0f

/* Clamp the feed-forward output. */
#define DOB_OUTPUT_CLAMP        400.0f

/* ---- API ---- */

void DOB_Init(DOB_Observer *dob, float motor_gain, float obs_gain, float ff_gain)
{
    memset(dob, 0, sizeof(*dob));

    dob->motor_gain    = (motor_gain > 0.001f) ? motor_gain : DEFAULT_MOTOR_GAIN;
    dob->observer_gain = (obs_gain   > 0.001f) ? obs_gain   : DEFAULT_OBSERVER_GAIN;
    dob->ff_gain       = (ff_gain    > 0.001f) ? ff_gain    : DEFAULT_FF_GAIN;

    dob->disturb_est   = 0.0f;
    dob->last_speed    = 0.0f;
    dob->last_pwm      = 0.0f;
}

float DOB_Update(DOB_Observer *dob, float speed, float pwm)
{
    /* ---- Actual acceleration (pulses/tick²) ---- */
    float actual_accel = speed - dob->last_speed;

    /* ---- Predicted acceleration from PWM (pulses/tick²) ---- */
    float predicted_accel = dob->motor_gain * dob->last_pwm;

    /* ---- Innovation: how much the real acceleration differs from model ---- */
    float innovation = actual_accel - predicted_accel;

    /* ---- 1st-order low-pass filter → disturbance estimate ---- */
    dob->disturb_est += dob->observer_gain * (innovation - dob->disturb_est);

    /* Clamp disturbance estimate */
    if (dob->disturb_est >  DOB_CLAMP) dob->disturb_est =  DOB_CLAMP;
    if (dob->disturb_est < -DOB_CLAMP) dob->disturb_est = -DOB_CLAMP;

    /* ---- Store state for next tick ---- */
    dob->last_speed = speed;
    dob->last_pwm   = pwm;

    /* ---- Feed-forward = estimated disturbance * ff_gain ---- */
    float ff = dob->ff_gain * dob->disturb_est;

    /* Clamp output */
    if (ff >  DOB_OUTPUT_CLAMP) ff =  DOB_OUTPUT_CLAMP;
    if (ff < -DOB_OUTPUT_CLAMP) ff = -DOB_OUTPUT_CLAMP;

    return ff;
}

void DOB_Reset(DOB_Observer *dob)
{
    dob->disturb_est *= 0.80f;   /* gentle decay, not instant reset */
    dob->last_speed   = 0.0f;
    dob->last_pwm     = 0.0f;
}

float DOB_Get_Disturbance(const DOB_Observer *dob)
{
    return dob->disturb_est;
}
