/**
 * pid_autotune.c — Relay-feedback PID auto-tuning implementation.
 *
 * References:
 *   Åström & Hägglund (1984) "Automatic Tuning of Simple Regulators"
 *   Ziegler & Nichols (1942) "Optimum Settings for Automatic Controllers"
 *
 * Units convention (matches the rest of the firmware):
 *   speed      — encoder pulses per 10 ms PID tick  (~11035 pulses/m)
 *   PWM        — 0..999  (TIM1 ARR)
 *   period     — PID ticks (10 ms each)
 *   gain Ku    — PWM / pulse  (how many PWM units per pulse of error)
 */
#include "pid_autotune.h"
#include "pid.h"
#include "bsp_encoder.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- Tunable constants ---- */

/* How many full oscillation cycles to measure (after skipping transients). */
#define MEASURE_CYCLES      6
#define SKIP_HALF_CYCLES    4    /* first 2 full cycles are transient */

/* Timeout per phase in PID ticks (100 Hz).
 * Prevents hanging if the motor cannot reach the test speed. */
#define ACCEL_TIMEOUT_TICKS   300   /* 3 seconds */
#define RELAY_TIMEOUT_TICKS   500   /* 5 seconds */
#define MEASURE_TIMEOUT_TICKS 800   /* 8 seconds */

/* Minimum acceptable ultimate period (ticks).
 * If Tu < MIN_TU the load may be too light (wheels in the air → meaningless). */
#define MIN_TU               6     /* 60 ms */

/* Default relay amplitude as fraction of max PWM. */
#define DEFAULT_RELAY_FRAC   0.25f   /* 25% of 999 ≈ 250 PWM */

/* ---- Helpers ---- */

/* Clamp x to [lo, hi]. */
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Estimate a reasonable relay amplitude based on test speed and zone. */
static uint16_t estimate_relay_amp(float test_speed, int zone) {
    (void)zone;
    uint16_t amp;

    /* Base: 25 % of full PWM range.  Scale up for higher speeds. */
    if (test_speed < 5.0f)       amp = 120;   /* STOP zone — gentle */
    else if (test_speed < 15.0f) amp = 180;   /* LOW zone */
    else if (test_speed < 60.0f) amp = 250;   /* MED zone */
    else                         amp = 350;   /* HIGH zone */

    if (amp > 500) amp = 500;  /* hard cap: 50 % PWM */
    return amp;
}

/* Estimate hysteresis band: 10 % of test speed, min 1.5 pulses. */
static float estimate_hyst(float test_speed) {
    float h = test_speed * 0.10f;
    if (h < 1.5f) h = 1.5f;
    if (h > 20.0f) h = 20.0f;
    return h;
}

/* ---- API ---- */

void AT_Init(PID_AutoTuner *at) {
    /* Zero everything — safe idle state. */
    for (unsigned i = 0; i < sizeof(*at); i++) {
        ((uint8_t *)at)[i] = 0;
    }
    at->state = AT_IDLE;
}

int AT_Start(PID_AutoTuner *at, float speed_pulses, int zone, float conservativeness) {
    if (at->state != AT_IDLE && at->state != AT_DONE && at->state != AT_ERROR) {
        return 0;  /* busy */
    }

    /* Clamp zone */
    if (zone < PID_ZONE_STOP) zone = PID_ZONE_STOP;
    if (zone > PID_ZONE_HIGH) zone = PID_ZONE_HIGH;

    /* Clamp conservativeness */
    if (conservativeness < 0.3f)  conservativeness = 0.3f;
    if (conservativeness > 2.0f)  conservativeness = 2.0f;

    /* Absolute speed (tuner always works with positive target;
     * direction is handled by relay sign). */
    float abs_speed = (speed_pulses >= 0.0f) ? speed_pulses : -speed_pulses;
    if (abs_speed < 0.5f) abs_speed = 0.5f;  /* absolute minimum */

    at->test_speed       = abs_speed;
    at->target_zone      = zone;
    at->conservativeness = conservativeness;
    at->relay_amp        = estimate_relay_amp(abs_speed, zone);
    at->relay_hyst       = estimate_hyst(abs_speed);

    /* Reset measurement state */
    at->ramped_target    = 0.0f;
    at->peak_high        = 0.0f;
    at->peak_low         = 0.0f;
    at->prev_err_sign    = 0;
    at->cross_tick       = 0;
    at->half_cycles      = 0;
    at->sum_period       = 0.0f;
    at->sum_amplitude    = 0.0f;
    at->relay_dir        = (speed_pulses >= 0.0f) ? 1 : -1;
    at->pwm_left         = 0;
    at->pwm_right        = 0;
    at->Ku = at->Tu = 0.0f;
    at->tuned_Kp = at->tuned_Ki = at->tuned_Kd = 0.0f;
    at->error_msg        = NULL;

    /* Go! */
    at->state       = AT_ACCEL;
    at->enter_tick  = HAL_GetTick();

    printf("\r\n[AUTOTUNE] Start - speed=%.1f p/tick (%.3f m/s)  zone=%d  relay_amp=%u  conserv=%.1f\r\n",
           (double)abs_speed, (double)(abs_speed * 100000.0f / ENCODER_PULSES_PER_METER / 1000.0f),
           zone, (unsigned)at->relay_amp, (double)conservativeness);
    printf("[AUTOTUNE] Accel phase - driving toward target...\r\n");

    return 1;
}

int AT_Tick(PID_AutoTuner *at, float speed) {
    if (at->state == AT_IDLE || at->state == AT_DONE || at->state == AT_ERROR) {
        return 0;  /* not active */
    }

    uint32_t now       = HAL_GetTick();
    uint32_t elapsed   = now - at->enter_tick;
    uint32_t ticks     = elapsed / 10;  /* approx PID ticks */

    float error = speed - at->test_speed;
    int8_t err_sign = (error > at->relay_hyst) ? 1 :
                      (error < -at->relay_hyst) ? -1 : 0;

    switch (at->state) {

    /* =================================================================
     * AT_ACCEL — drive toward test speed with fixed PWM.
     * ================================================================= */
    case AT_ACCEL: {
        /* Soft-ramp the target for monitoring, but use fixed PWM for drive. */
        if (at->ramped_target < at->test_speed - 3.0f) {
            at->ramped_target += 3.0f;
        } else {
            at->ramped_target = at->test_speed;
        }

        /* Fixed PWM in the target direction. */
        int16_t drive = (int16_t)at->relay_amp * at->relay_dir;
        at->pwm_left  = drive;
        at->pwm_right = drive;

        /* Transition when speed crosses the test_speed (with hysteresis) */
        if (speed >= at->test_speed) {
            at->state      = AT_RELAY;
            at->enter_tick = now;
            at->peak_high  = speed;
            at->peak_low   = speed;
            at->prev_err_sign = 0;
            at->half_cycles   = 0;
            printf("[AUTOTUNE] Relay phase - oscillating (skip %d transient cycles)...\r\n",
                   SKIP_HALF_CYCLES / 2);
        } else if (ticks > ACCEL_TIMEOUT_TICKS) {
            at->state     = AT_ERROR;
            at->error_msg = "Timeout: motor cannot reach test speed";
            printf("[AUTOTUNE] ERROR: %s\r\n", at->error_msg);
        }
        break;
    }

    /* =================================================================
     * AT_RELAY — relay control, skip transient cycles.
     * ================================================================= */
    case AT_RELAY: {
        /* Relay control law */
        if (speed > at->test_speed + at->relay_hyst) {
            /* Overshooting — reverse direction */
            at->pwm_left  = -(int16_t)at->relay_amp * at->relay_dir;
            at->pwm_right = at->pwm_left;
        } else if (speed < at->test_speed - at->relay_hyst) {
            /* Undershooting — forward direction */
            at->pwm_left  = (int16_t)at->relay_amp * at->relay_dir;
            at->pwm_right = at->pwm_left;
        }
        /* else: keep last PWM (hysteresis dead-band) */

        /* Peak tracking */
        if (error > 0.0f && speed > at->peak_high) {
            at->peak_high = speed;
        }
        if (error < 0.0f && speed < at->peak_low) {
            at->peak_low = speed;
        }

        /* Zero-crossing detection: error sign changed */
        if (err_sign != 0 && err_sign != at->prev_err_sign && at->prev_err_sign != 0) {
            /* We just crossed through the hysteresis band — a half-cycle completed. */
            at->half_cycles++;

            if (at->half_cycles > SKIP_HALF_CYCLES) {
                /* This half-cycle is part of a valid measurement. */
                /* On even crossings (full cycle complete), record period + amplitude. */
                if ((at->half_cycles & 1) == 0) {
                    /* Full cycle complete. */
                    uint32_t cycle_ticks = (now - at->cross_tick) / 10;
                    if (cycle_ticks < 1) cycle_ticks = 1;

                    float p2p = at->peak_high - at->peak_low;
                    if (p2p < 0.5f) p2p = 0.5f;  /* floor to avoid noise inflation */

                    at->sum_period    += (float)cycle_ticks;
                    at->sum_amplitude += p2p;

                    #if 0  /* verbose debug — enable for troubleshooting */
                    printf("  [AT] cycle: T=%lu  pk2pk=%.1f  Ku_est=%.1f\r\n",
                           (unsigned long)cycle_ticks, (double)p2p,
                           (double)(4.0f * at->relay_amp / (M_PI * p2p * 0.5f)));
                    #endif
                }
                /* Reset peaks for next half-cycle */
                at->peak_high = speed;
                at->peak_low  = speed;
                at->cross_tick = now;
            }

            if (at->half_cycles >= SKIP_HALF_CYCLES + MEASURE_CYCLES * 2) {
                /* Enough data — compute gains. */
                at->state      = AT_COMPUTE;
                at->enter_tick = now;
            }
        }
        at->prev_err_sign = err_sign;

        /* Also advance to MEASURE state after enough half-cycles (labels the phase). */
        if (at->half_cycles >= SKIP_HALF_CYCLES && at->state == AT_RELAY) {
            at->state      = AT_MEASURE;
            at->enter_tick = now;
            printf("[AUTOTUNE] Measure phase - collecting %d full cycles...\r\n", MEASURE_CYCLES);
        }

        /* Timeout */
        if (ticks > RELAY_TIMEOUT_TICKS && at->half_cycles < SKIP_HALF_CYCLES) {
            at->state     = AT_ERROR;
            at->error_msg = "Timeout: relay oscillation did not stabilise";
            printf("[AUTOTUNE] ERROR: %s\r\n", at->error_msg);
        }
        break;
    }

    /* =================================================================
     * AT_MEASURE — relay control, collecting data (logic handled in AT_RELAY).
     *              This state is just a label; the actual work is above.
     *              The transition to AT_COMPUTE happens inside AT_RELAY.
     * ================================================================= */
    case AT_MEASURE: {
        /* Same relay logic as AT_RELAY. */
        if (speed > at->test_speed + at->relay_hyst) {
            at->pwm_left  = -(int16_t)at->relay_amp * at->relay_dir;
            at->pwm_right = at->pwm_left;
        } else if (speed < at->test_speed - at->relay_hyst) {
            at->pwm_left  = (int16_t)at->relay_amp * at->relay_dir;
            at->pwm_right = at->pwm_left;
        }

        /* Peak tracking */
        if (error > 0.0f && speed > at->peak_high) at->peak_high = speed;
        if (error < 0.0f && speed < at->peak_low)  at->peak_low  = speed;

        /* Zero-crossing detection */
        if (err_sign != 0 && err_sign != at->prev_err_sign && at->prev_err_sign != 0) {
            at->half_cycles++;
            if ((at->half_cycles & 1) == 0) {
                uint32_t cycle_ticks = (now - at->cross_tick) / 10;
                if (cycle_ticks < 1) cycle_ticks = 1;
                float p2p = at->peak_high - at->peak_low;
                if (p2p < 0.5f) p2p = 0.5f;
                at->sum_period    += (float)cycle_ticks;
                at->sum_amplitude += p2p;
            }
            at->peak_high = speed;
            at->peak_low  = speed;
            at->cross_tick = now;

            if (at->half_cycles >= SKIP_HALF_CYCLES + MEASURE_CYCLES * 2) {
                at->state      = AT_COMPUTE;
                at->enter_tick = now;
            }
        }
        at->prev_err_sign = err_sign;

        /* Timeout */
        if (ticks > MEASURE_TIMEOUT_TICKS) {
            at->state     = AT_ERROR;
            at->error_msg = "Timeout: measurement phase took too long";
            printf("[AUTOTUNE] ERROR: %s\r\n", at->error_msg);
        }
        break;
    }

    /* =================================================================
     * AT_COMPUTE — crunch the numbers.
     * ================================================================= */
    case AT_COMPUTE: {
        int n_cycles = MEASURE_CYCLES;
        float avg_T  = at->sum_period    / (float)n_cycles;   /* ticks */
        float avg_p2p = at->sum_amplitude / (float)n_cycles;   /* pulses */

        if (avg_T < 1.0f)  avg_T  = 1.0f;
        if (avg_p2p < 1.0f) avg_p2p = 1.0f;

        float a = avg_p2p * 0.5f;   /* half-amplitude (pulses) */

        /* Ultimate gain (describing-function approximation of the relay).
         * Ku = 4d / (π·a)  where d = relay amplitude (PWM), a = half-amplitude (pulses). */
        at->Ku = (4.0f * (float)at->relay_amp) / (M_PI * a);
        at->Tu = avg_T;

        /* Sanity checks */
        if (at->Tu < (float)MIN_TU) {
            printf("[AUTOTUNE] WARNING: Tu=%.1f ticks (<%d ms).  "
                   "Load may be too light (wheels in the air?).  "
                   "Gains will be conservative.\r\n",
                   (double)at->Tu, MIN_TU * 10);
            at->Tu = (float)MIN_TU;   /* clamp to minimum */
        }

        float C = at->conservativeness;

        /* Ziegler–Nichols PID tuning rules:
         *   Kp = 0.6 · Ku
         *   Ti = 0.5 · Tu   →   Ki = Kp / Ti = 1.2 · Ku / Tu
         *   Td = 0.125 · Tu  →   Kd = Kp · Td = 0.075 · Ku · Tu
         *
         * Conservativeness factor C scales all gains proportionally:
         *   C=0.5: safe, less overshoot
         *   C=1.0: standard ZN (may have ~25 % overshoot)
         *   C=1.5: aggressive, faster response
         */
        at->tuned_Kp = C * 0.60f * at->Ku;
        at->tuned_Ki = C * 1.20f * at->Ku / at->Tu;
        at->tuned_Kd = C * 0.075f * at->Ku * at->Tu;

        /* Clamp gains to reasonable ranges */
        at->tuned_Kp = clampf(at->tuned_Kp, 0.3f, 8.0f);
        at->tuned_Ki = clampf(at->tuned_Ki, 0.02f, 2.0f);
        at->tuned_Kd = clampf(at->tuned_Kd, 0.02f, 1.5f);

        /* Apply to the gain table immediately (table is now mutable). */
        PID_GainEntry *entry = PID_Get_Gain_Entry_Mutable(at->target_zone);
        if (entry) {
            entry->Kp = at->tuned_Kp;
            entry->Ki = at->tuned_Ki;
            entry->Kd = at->tuned_Kd;

            /* Derive secondary parameters from the tuned values:
             *   integral_sep ≈ 3 × test-speed amplitude  → covers normal error
             *   slew_max ≈ relay_amp / 2  → half the relay step per tick
             *   encoder_alpha: moderate filter (zone-dependent) */
            entry->integral_sep   = at->test_speed * 0.4f;   /* 40 % of test speed */
            entry->slew_max       = at->relay_amp / 2;
            entry->encoder_alpha  = (at->target_zone == PID_ZONE_STOP) ? 0.20f :
                                    (at->target_zone == PID_ZONE_LOW)  ? 0.30f :
                                    (at->target_zone == PID_ZONE_MED)  ? 0.40f :
                                                                         0.50f;
        }

        /* Print results */
        printf("\r\n[AUTOTUNE] ====== TUNING COMPLETE ======\r\n");
        printf("[AUTOTUNE] Relay amp  : %u PWM\r\n", (unsigned)at->relay_amp);
        printf("[AUTOTUNE] Osc amp    : %.1f pulses pk-pk  (%.1f half)\r\n",
               (double)avg_p2p, (double)a);
        printf("[AUTOTUNE] Period Tu  : %.1f ticks (%.0f ms)\r\n",
               (double)at->Tu, (double)(at->Tu * 10.0f));
        printf("[AUTOTUNE] Gain Ku    : %.2f PWM/pulse\r\n", (double)at->Ku);
        printf("[AUTOTUNE] Tuned Kp   : %.3f\r\n", (double)at->tuned_Kp);
        printf("[AUTOTUNE] Tuned Ki   : %.3f\r\n", (double)at->tuned_Ki);
        printf("[AUTOTUNE] Tuned Kd   : %.3f\r\n", (double)at->tuned_Kd);
        printf("[AUTOTUNE] integral_sep: %.1f\r\n", (double)(at->test_speed * 0.4f));
        printf("[AUTOTUNE] Zone %d updated.  Type STATUS to verify.\r\n",
               at->target_zone);
        printf("[AUTOTUNE] ================================\r\n\r\n");

        at->state      = AT_DONE;
        at->enter_tick = now;
        /* Stop motors */
        at->pwm_left  = 0;
        at->pwm_right = 0;
        break;
    }

    default:
        break;
    }

    return AT_Is_Active(at);
}

int AT_Is_Done(const PID_AutoTuner *at) {
    return (at->state == AT_DONE || at->state == AT_ERROR) ? 1 : 0;
}

int AT_Is_Active(const PID_AutoTuner *at) {
    return (at->state != AT_IDLE &&
            at->state != AT_DONE &&
            at->state != AT_ERROR) ? 1 : 0;
}

const char *AT_State_Name(AT_State s) {
    switch (s) {
        case AT_IDLE:       return "IDLE";
        case AT_ACCEL:      return "ACCEL";
        case AT_WAIT_STEADY:return "WAIT_STEADY";
        case AT_RELAY:      return "RELAY";
        case AT_MEASURE:    return "MEASURE";
        case AT_COMPUTE:    return "COMPUTE";
        case AT_DONE:       return "DONE";
        case AT_ERROR:      return "ERROR";
        default:            return "?";
    }
}

void AT_Get_Results(const PID_AutoTuner *at,
                    float *Kp, float *Ki, float *Kd,
                    float *Ku, float *Tu) {
    if (Kp) *Kp = at->tuned_Kp;
    if (Ki) *Ki = at->tuned_Ki;
    if (Kd) *Kd = at->tuned_Kd;
    if (Ku) *Ku = at->Ku;
    if (Tu) *Tu = at->Tu;
}

void AT_Abort(PID_AutoTuner *at) {
    printf("[AUTOTUNE] Aborted by user.\r\n");
    at->state      = AT_IDLE;
    at->pwm_left   = 0;
    at->pwm_right  = 0;
}
