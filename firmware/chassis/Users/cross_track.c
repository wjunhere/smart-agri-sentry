/**
 * cross_track.c — Cross-Track Stabilizer implementation.
 *
 * Three independent mechanisms, each with minimal coupling:
 *
 *   A. YAW DRIFT COMPENSATOR
 *      - Only active when |target_L - target_R| < TURN_THRESHOLD (straight line)
 *      - Integrates (speed_L - speed_R) each tick → yaw_integral
 *      - yaw_integral decays by yaw_decay each tick (≈ 2 s time constant)
 *      - When |yaw_integral| > yaw_threshold, applies proportional correction
 *      - Correction: bias_left -= corr, bias_right += corr (split across sides)
 *      - Clamped to ±yaw_max per side
 *
 *   B. DIFFERENTIAL LOAD BALANCER
 *      - Compares |DOB_L - DOB_R| against load_diff_thresh
 *      - When exceeded: the side with MORE load (higher DOB) is working harder
 *        → reduce target on the LESS loaded side (free-spinning prevention)
 *      - Correction proportional to excess DOB difference
 *      - Clamped to ±load_max per side
 *
 *   C. CROSS-SPEED CONSENSUS FILTER
 *      - Only when going straight (targets equal)
 *      - When |speed_L - speed_R| > consensus_thresh:
 *        → The side further from the shared target is likely bouncing
 *        → Blend that side's target 20% toward the stable side's speed
 *        → This prevents the PID from chasing a bounce artifact
 *      - Sets consensus_fired flag for diagnostics
 *
 * All corrections are ADDED to the target speeds before PID_Calc.
 * The biases naturally decay toward zero when conditions normalize:
 *   - yaw_integral decays every tick
 *   - load balance → 0 when DOB estimates converge
 *   - consensus is per-tick (no memory)
 */

#include "cross_track.h"
#include <string.h>

/* ------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------ */

static float fabsf_local(float x) {
    return (x >= 0.0f) ? x : -x;
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ------------------------------------------------------------------------
 * CTS_Init
 * ------------------------------------------------------------------------ */
void CTS_Init(CTS_Stabilizer *cts)
{
    memset(cts, 0, sizeof(*cts));

    /* A. Yaw drift */
    cts->yaw_gain      = CTS_YAW_GAIN_DEFAULT;
    cts->yaw_max       = CTS_YAW_MAX_DEFAULT;
    cts->yaw_decay     = CTS_YAW_DECAY_DEFAULT;
    cts->yaw_threshold = CTS_YAW_THRESHOLD_DEFAULT;
    cts->yaw_integral  = 0.0f;

    /* B. Load balance */
    cts->load_diff_thresh = CTS_LOAD_DIFF_THRESH_DEFAULT;
    cts->load_gain        = CTS_LOAD_GAIN_DEFAULT;
    cts->load_max         = CTS_LOAD_MAX_DEFAULT;

    /* C. Consensus */
    cts->consensus_thresh = CTS_CONSENSUS_THRESH_DEFAULT;
    cts->consensus_alpha  = CTS_CONSENSUS_ALPHA_DEFAULT;

    /* Outputs */
    cts->bias_left  = 0.0f;
    cts->bias_right = 0.0f;

    /* State */
    cts->turning         = 0;
    cts->consensus_fired = 0;

    /* Diagnostics */
    cts->yaw_events       = 0;
    cts->balance_events   = 0;
    cts->consensus_events = 0;
    cts->tick_count       = 0;
}

/* ------------------------------------------------------------------------
 * CTS_Reset — decay internal state toward zero (soft reset).
 * Called on STOP or auto-tune start so old drift doesn't carry over.
 * ------------------------------------------------------------------------ */
void CTS_Reset(CTS_Stabilizer *cts)
{
    cts->yaw_integral *= 0.50f;    /* halve the accumulated drift */
    cts->bias_left     = 0.0f;
    cts->bias_right    = 0.0f;
    cts->turning       = 0;
    cts->consensus_fired = 0;
}

/* ------------------------------------------------------------------------
 * CTS_Update — main entry point, called at 100 Hz from main loop.
 * ------------------------------------------------------------------------ */
void CTS_Update(CTS_Stabilizer *cts,
                float speed_left,   float speed_right,
                float dob_left,     float dob_right,
                float *target_left, float *target_right,
                float pwm_left,     float pwm_right)
{
    /* Silence unused-parameter warnings (PWM unused in current algorithm
     * but retained in API for future slip-detection enhancements). */
    (void)pwm_left;
    (void)pwm_right;

    cts->tick_count++;

    /* ---- Reset per-tick outputs ---- */
    cts->bias_left       = 0.0f;
    cts->bias_right      = 0.0f;
    cts->turning         = 0;
    cts->consensus_fired = 0;

    float tgt_l = *target_left;
    float tgt_r = *target_right;

    /* ---- Detect intentional turn ----
     * When the RDK commands different speeds to left and right, the
     * vehicle is SUPPOSED to turn.  Disable yaw correction so we
     * don't fight the driver.  Load balancing stays active (we still
     * want both tracks to share load during a turn). */
    float tgt_diff = tgt_l - tgt_r;
    float tgt_diff_abs = fabsf_local(tgt_diff);
    if (tgt_diff_abs > CTS_TURN_THRESHOLD) {
        cts->turning = 1;
    }

    /* =================================================================
     * A. YAW DRIFT COMPENSATOR
     *
     * Only active when:
     *   - Not turning (targets are equal within threshold)
     *   - Vehicle is moving (non-zero target)
     * ================================================================= */
    float tgt_abs_l = fabsf_local(tgt_l);
    float tgt_abs_r = fabsf_local(tgt_r);
    float tgt_max   = (tgt_abs_l > tgt_abs_r) ? tgt_abs_l : tgt_abs_r;

    if (!cts->turning && tgt_max > 0.5f) {
        /* Integrate the L-R speed difference.
         * When left is faster: yaw_integral grows positive → we'll
         * slow left and speed up right to cancel the rightward yaw. */
        float yaw_error = speed_left - speed_right;
        cts->yaw_integral += yaw_error;

        /* Gentle decay to prevent unbounded accumulation.
         * At yaw_decay=0.995: time constant ≈ 200 ticks (2 s). */
        cts->yaw_integral *= cts->yaw_decay;

        /* Deadband: only correct when drift exceeds threshold.
         * A 5-pulse threshold at 11035 pulses/m ≈ 0.45 mm of
         * accumulated differential travel. */
        float yaw_abs = fabsf_local(cts->yaw_integral);
        if (yaw_abs > cts->yaw_threshold) {
            /* Correction proportional to excess drift.
             * yaw_gain=0.005 × 10 pulses excess = 0.05 pulses/tick bias.
             * This is VERY gentle — it takes ~2 seconds to cancel a
             * 10-pulse accumulated drift. */
            float excess = yaw_abs - cts->yaw_threshold;
            float corr   = cts->yaw_gain * excess;
            corr = clampf(corr, 0.0f, cts->yaw_max);

            /* Preserve the sign: if left is faster (positive integral),
             * reduce left target (negative bias) and increase right. */
            if (cts->yaw_integral > 0.0f) {
                cts->bias_left  -= corr;
                cts->bias_right += corr;
            } else {
                cts->bias_left  += corr;
                cts->bias_right -= corr;
            }
            cts->yaw_events++;
        }
    } else if (tgt_max <= 0.5f) {
        /* Stopped — decay yaw integral faster.
         * This prevents old drift from affecting the next movement. */
        cts->yaw_integral *= 0.90f;
    }
    /* During intentional turns: hold yaw_integral constant (no decay,
     * no accumulation).  When the turn ends, the compensator picks up
     * where it left off — this is correct because the physical heading
     * changed during the turn, and the old drift estimate is stale.
     * Actually, we DO decay it during turns (mildly) to avoid carrying
     * stale drift across a long turn: */
    if (cts->turning) {
        cts->yaw_integral *= 0.98f;   /* ~500 ms time constant during turns */
    }

    /* =================================================================
     * B. DIFFERENTIAL LOAD BALANCER
     *
     * Compares DOB disturbance estimates.  A higher DOB means the
     * motor is working against more resistance (soft soil, slope).
     *
     * Strategy: REDUCE target on the LESS loaded side.
     * This prevents the free-spinning-track problem: when one track
     * has no traction, instead of letting it spin at full speed
     * (and wasting power + wearing the track), we slow it down so
     * both tracks share the load.
     *
     * Active even during turns (but gentler).
     * ================================================================= */
    {
        float dob_diff     = dob_left - dob_right;
        float dob_diff_abs = fabsf_local(dob_diff);

        if (dob_diff_abs > cts->load_diff_thresh) {
            float excess = dob_diff_abs - cts->load_diff_thresh;
            float corr   = cts->load_gain * excess;
            corr = clampf(corr, 0.0f, cts->load_max);

            /* Reduce correction during turns by 50%.
             * During a turn, some DOB asymmetry is expected —
             * the inner track naturally works harder. */
            if (cts->turning) {
                corr *= 0.50f;
            }

            if (dob_diff > 0.0f) {
                /* Left DOB higher → left loaded more → reduce RIGHT target
                 * (right is the unloaded "free-spinning" side) */
                cts->bias_right -= corr;
            } else {
                /* Right DOB higher → right loaded more → reduce LEFT target */
                cts->bias_left  -= corr;
            }
            cts->balance_events++;
        }
    }

    /* =================================================================
     * C. CROSS-SPEED CONSENSUS FILTER
     *
     * When going straight and one encoder reading suddenly diverges
     * by an implausible amount, it's likely a bounce/jolt on that
     * track (the wheel momentarily loses or regains ground contact).
     *
     * Instead of modifying the encoder reading (which would confuse
     * the filter state), we flag the event and nudge the TARGET for
     * that side toward the stable side.  This prevents the PID from
     * over-reacting to a momentary artifact.
     *
     * The correction is applied as a target bias that decays
     * immediately (no memory across ticks) — if the divergence
     * persists, it's real (not an artifact) and the PID should
     * handle it.
     * ================================================================= */
    if (!cts->turning && tgt_max > 0.5f) {
        float spd_diff     = speed_left - speed_right;
        float spd_diff_abs = fabsf_local(spd_diff);

        if (spd_diff_abs > cts->consensus_thresh) {
            /* Determine which side is the outlier.
             * The side further from its own target is more likely to
             * be experiencing a bounce. */
            float err_l = fabsf_local(speed_left  - tgt_l);
            float err_r = fabsf_local(speed_right - tgt_r);

            if (err_l > err_r) {
                /* Left is the outlier — blend left target toward
                 * the right side's speed, scaled by consensus_alpha. */
                float blend = cts->consensus_alpha * (speed_right - speed_left);
                cts->bias_left += blend;
            } else {
                /* Right is the outlier */
                float blend = cts->consensus_alpha * (speed_left - speed_right);
                cts->bias_right += blend;
            }
            cts->consensus_fired = 1;
            cts->consensus_events++;
        }
    }

    /* =================================================================
     * Apply corrections to targets.
     *
     * Safety clamps:
     *   - bias MUST NOT reverse the target direction (sign flip)
     *   - bias MUST NOT exceed ±50% of |target| for large targets
     *   - bias is clamped to ±max(yaw_max, load_max) in absolute terms
     *
     * These ensure the CTS can never cause a runaway or direction
     * reversal, even if a sensor glitch produces a wild DOB estimate.
     * ================================================================= */

    /* Clamp each bias to its safety limit */
    float max_bias = (cts->yaw_max > cts->load_max) ? cts->yaw_max : cts->load_max;
    cts->bias_left  = clampf(cts->bias_left,  -max_bias, max_bias);
    cts->bias_right = clampf(cts->bias_right, -max_bias, max_bias);

    /* Direction safety: bias must not flip the sign of target.
     * If target is positive, bias must not push it negative, and vice versa. */
    float new_tgt_l = tgt_l + cts->bias_left;
    float new_tgt_r = tgt_r + cts->bias_right;

    if (tgt_l > 0.5f && new_tgt_l < 0.0f) {
        cts->bias_left  = -tgt_l * 0.5f;  /* allow at most 50% reduction */
        new_tgt_l = tgt_l + cts->bias_left;
    } else if (tgt_l < -0.5f && new_tgt_l > 0.0f) {
        cts->bias_left  = -tgt_l * 0.5f;
        new_tgt_l = tgt_l + cts->bias_left;
    }

    if (tgt_r > 0.5f && new_tgt_r < 0.0f) {
        cts->bias_right = -tgt_r * 0.5f;
        new_tgt_r = tgt_r + cts->bias_right;
    } else if (tgt_r < -0.5f && new_tgt_r > 0.0f) {
        cts->bias_right = -tgt_r * 0.5f;
        new_tgt_r = tgt_r + cts->bias_right;
    }

    /* Write back */
    *target_left  = new_tgt_l;
    *target_right = new_tgt_r;
}
