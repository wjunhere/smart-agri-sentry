/**
 * cross_track.h — Cross-Track Stabilizer (CTS) for tracked vehicles.
 *
 * Farmland Problem:
 *   Left and right tracks often encounter DIFFERENT ground conditions:
 *   one side on hard packed soil, the other in loose dirt, mud, or a rut.
 *   This causes:
 *     1. Yaw drift — vehicle veers off course even with equal target speeds
 *     2. Asymmetric load — one motor works harder, the other spins freely
 *     3. Bounce artifacts — one track loses ground contact momentarily
 *
 * Solution — three complementary mechanisms, all running at 100 Hz:
 *
 *   A. YAW DRIFT COMPENSATOR
 *      - Accumulates L-R speed difference over time (virtual heading integrator)
 *      - When cumulative drift exceeds threshold, gently biases targets to
 *        slow the faster side and speed up the slower side
 *      - Disabled during intentional turns (|target_L - target_R| > threshold)
 *      - Slow time constant — doesn't fight the driver, just cancels drift
 *
 *   B. DIFFERENTIAL LOAD BALANCER
 *      - Compares DOB estimates between left and right motors
 *      - If one DOB >> the other, that track is loaded more (soft soil, slope)
 *      - Gently reduces target speed on the UNLOADED side
 *      - This prevents the free-spinning-track problem and keeps both
 *        motors operating at similar load points
 *
 *   C. CROSS-SPEED CONSENSUS FILTER
 *      - When traveling straight and one encoder reading suddenly diverges
 *        from the other by a physically implausible amount (bounce/jolt),
 *        the outlier is flagged and nudged toward the consensus value
 *      - The encoder's adaptive filter catches single-sample glitches;
 *        this catches sustained (2-5 tick) bounce oscillations
 *
 * All corrections are SMALL and SLOW — typically 0.5-3 pulses/10ms
 * (0.05-0.27 m/s).  They don't fight intentional control; they just
 * cancel the asymmetry that uneven terrain introduces.
 */

#ifndef __CROSS_TRACK_H
#define __CROSS_TRACK_H
#include "stm32f4xx_hal.h"

/* =========================================================================
 * Stabilizer state — one instance for the whole vehicle (covers both tracks)
 * ========================================================================= */
typedef struct {
    /* ---- A. Yaw drift compensator ---- */
    float   yaw_gain;         /* correction gain (0.002–0.01) */
    float   yaw_max;          /* max correction per side, pulses/10ms */
    float   yaw_decay;        /* per-tick decay when targets equal (0.990–0.999) */
    float   yaw_threshold;    /* min |yaw_integral| to fire correction, pulses */
    float   yaw_integral;     /* accumulator: sum of (speed_L - speed_R), pulses */

    /* ---- B. Differential load balancer ---- */
    float   load_diff_thresh; /* min |DOB_L - DOB_R| to trigger, PWM units */
    float   load_gain;        /* DOB diff → target correction gain */
    float   load_max;         /* max correction per side, pulses/10ms */

    /* ---- C. Cross-speed consensus ---- */
    float   consensus_thresh; /* max credible |speed_L - speed_R| when straight */
    float   consensus_alpha;  /* blend factor toward consensus (0.10–0.30) */

    /* ---- Correction outputs (read by main.c each tick) ---- */
    float   bias_left;        /* target correction for left track, pulses/10ms */
    float   bias_right;       /* target correction for right track, pulses/10ms */

    /* ---- State (internal) ---- */
    uint8_t turning;          /* flag: intentional turn detected this tick */
    uint8_t consensus_fired;  /* flag: consensus filter activated this tick */

    /* ---- Diagnostics ---- */
    uint16_t yaw_events;      /* count: yaw corrections applied */
    uint16_t balance_events;  /* count: load-balance corrections applied */
    uint16_t consensus_events;/* count: consensus filter activations */
    uint16_t tick_count;      /* total ticks processed */
} CTS_Stabilizer;

/* =========================================================================
 * Default configuration — tuned for farmland at 0.3–0.6 m/s
 *
 * These values are conservative: corrections are gentle (1–3 pulses/tick)
 * and the time constants are slow (200 ms–2 s).  The idea is to cancel
 * drift without fighting the driver or creating instability.
 * ========================================================================= */

/* Yaw: ~2 s time constant.  A 5-pulse accumulated drift corrects at
 * 0.005 × 5 = 0.025 pulses/tick → ~200 ticks (2 s) to zero out. */
#define CTS_YAW_GAIN_DEFAULT         0.005f
#define CTS_YAW_MAX_DEFAULT           3.0f   /* 3 pulses ≈ 0.27 m/s */
#define CTS_YAW_DECAY_DEFAULT         0.995f /* ~2 s decay time constant */
#define CTS_YAW_THRESHOLD_DEFAULT     5.0f   /* 5 pulses drift → 0.45 m/s·tick */

/* Load balance: when DOB estimates differ by >30 PWM (~3% duty),
 * correct at 0.02 pulses/PWM → ~0.6 pulses/tick for a 30-PWM diff. */
#define CTS_LOAD_DIFF_THRESH_DEFAULT 30.0f
#define CTS_LOAD_GAIN_DEFAULT        0.02f
#define CTS_LOAD_MAX_DEFAULT          2.0f   /* 2 pulses ≈ 0.18 m/s */

/* Consensus: when straight-line speeds differ by >8 pulses (~0.73 m/s
 * at low speed), blend 20% toward the stable side. */
#define CTS_CONSENSUS_THRESH_DEFAULT  8.0f
#define CTS_CONSENSUS_ALPHA_DEFAULT   0.20f

/* When |target_L - target_R| > this, the vehicle is turning —
 * disable yaw correction.  (1.5 pulses ≈ 0.14 m/s difference.) */
#define CTS_TURN_THRESHOLD            1.5f

/* =========================================================================
 * API
 * ========================================================================= */

/* One-time init with safe defaults. */
void CTS_Init(CTS_Stabilizer *cts);

/* Run one PID tick (100 Hz).
 *
 * INPUT (read-only):
 *   speed_left/right  — filtered encoder speed, pulses/10ms
 *   dob_left/right    — DOB disturbance estimate, PWM units
 *   pwm_left/right    — current PWM output (from previous tick)
 *
 * INPUT/OUTPUT (modified in-place):
 *   target_left/right — desired speed, pulses/10ms.
 *     On output, CTS bias is ADDED to these values.
 *     Caller should use the modified targets in PID_Calc().
 *
 * The corrections are intentionally bounded (±yaw_max, ±load_max)
 * so they can't cause large speed deviations even in edge cases. */
void CTS_Update(CTS_Stabilizer *cts,
                float speed_left,   float speed_right,
                float dob_left,     float dob_right,
                float *target_left, float *target_right,
                float pwm_left,     float pwm_right);

/* Reset internal state (call on stop or auto-tune start). */
void CTS_Reset(CTS_Stabilizer *cts);

#endif /* __CROSS_TRACK_H */
