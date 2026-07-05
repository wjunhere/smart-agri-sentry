/**
 * pid_autotune.h — Relay-feedback PID auto-tuning (Åström–Hägglund method).
 *
 * How it works:
 *   1. Accelerate to a test speed using current PID.
 *   2. Switch to relay (bang-bang) control — oscillates around the setpoint.
 *   3. Measure oscillation period (Tu) and amplitude (a).
 *   4. Compute ultimate gain:  Ku = 4d / (π·a)   where d = relay amplitude.
 *   5. Apply Ziegler–Nichols / Tyreus–Luyben rules to compute Kp, Ki, Kd.
 *   6. Write the tuned gains into the global PID gain table.
 *
 * Trigger from debug console:  TUNE <speed_mm_s> [conservativeness]
 *   TUNE 300        → tune at 0.3 m/s, standard (1.0)
 *   TUNE 300 0.5    → tune at 0.3 m/s, conservative
 *   TUNE 600 1.5    → tune at 0.6 m/s, aggressive
 */
#ifndef __PID_AUTOTUNE_H
#define __PID_AUTOTUNE_H
#include "pid.h"
#include <stdint.h>

/* ---- Auto-tuner state machine ---- */
typedef enum {
    AT_IDLE = 0,
    AT_ACCEL,          /* ramp to test speed with current PID */
    AT_WAIT_STEADY,    /* let speed settle before relay switch */
    AT_RELAY,          /* relay (bang-bang) oscillation — skip first 2 cycles */
    AT_MEASURE,        /* measuring period + amplitude (6 full cycles) */
    AT_COMPUTE,        /* computing Ku, Tu → Kp/Ki/Kd */
    AT_DONE,           /* tuning complete, gains written */
    AT_ERROR           /* tuning failed (timeout / not enough motion) */
} AT_State;

/* ---- Per-instance auto-tuner (one for the whole chassis) ---- */
typedef struct {
    AT_State state;
    uint32_t enter_tick;       /* when we entered current state (HAL_GetTick) */

    /* --- Configuration --- */
    float    test_speed;       /* target test speed in pulses/10ms */
    uint16_t relay_amp;        /* relay output amplitude, PWM units */
    float    relay_hyst;       /* hysteresis band, pulses/10ms */
    int      target_zone;      /* which PID_ZONE_* to update */
    float    conservativeness; /* 0.5=gentle, 1.0=ZN standard, 1.5=aggressive */

    /* --- Ramp state (AT_ACCEL) --- */
    float    ramped_target;    /* soft-ramped test target */

    /* --- Relay / measure state --- */
    float    peak_high;        /* current half-cycle positive peak */
    float    peak_low;         /* current half-cycle negative peak */
    int8_t   prev_err_sign;    /* previous error sign for zero-cross detect */
    uint32_t cross_tick;       /* tick of last zero-crossing */
    int      half_cycles;      /* half-cycles measured (skip first 4 = 2 full) */
    float    sum_period;       /* accumulator: sum of full-cycle periods (ticks) */
    float    sum_amplitude;    /* accumulator: sum of peak-to-peak amplitudes */

    /* --- Relay direction --- */
    int8_t   relay_dir;        /* +1 or -1 */

    /* --- Output (set each tick, read by main loop) --- */
    int16_t  pwm_left;         /* current PWM to apply */
    int16_t  pwm_right;

    /* --- Results (valid when AT_DONE) --- */
    float    Ku;               /* ultimate gain (PWM / pulse) */
    float    Tu;               /* ultimate period (PID ticks, 10 ms) */
    float    tuned_Kp;
    float    tuned_Ki;
    float    tuned_Kd;

    /* --- Error message (valid when AT_ERROR) --- */
    const char *error_msg;
} PID_AutoTuner;

/* ---- API ---- */

/* One-time init (call from main, before loop). */
void AT_Init(PID_AutoTuner *at);

/* Start tuning at a given speed (pulses/10ms).
 * zone: PID_ZONE_STOP..HIGH to update; conservativeness: 0.5–1.5.
 * Returns 0 if auto-tuner is already busy. */
int AT_Start(PID_AutoTuner *at, float speed_pulses, int zone, float conservativeness);

/* Run one PID tick of the auto-tuner (called at 100 Hz from main loop).
 * speed: current filtered encoder speed (pulses/10ms), averaged from both wheels.
 * Returns 1 if auto-tuner is active (PID should be bypassed for motors). */
int AT_Tick(PID_AutoTuner *at, float speed);

/* True once the tuning sequence has finished (success OR error). */
int AT_Is_Done(const PID_AutoTuner *at);

/* True while the tuner is actively running (IDLE/DONE/ERROR → 0). */
int AT_Is_Active(const PID_AutoTuner *at);

/* Human-readable state name for printf. */
const char *AT_State_Name(AT_State s);

/* Read back results. Only call after AT_Is_Done() returns true. */
void AT_Get_Results(const PID_AutoTuner *at,
                    float *Kp, float *Ki, float *Kd,
                    float *Ku, float *Tu);

/* Cancel an in-progress tuning run. */
void AT_Abort(PID_AutoTuner *at);

#endif /* __PID_AUTOTUNE_H */
