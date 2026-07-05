#include "app_fsm.h"
#include "aggregator.h"
#include "lora_frame.h"
#include "lora_tx.h"
#include "status_led.h"
#include "cj702.h"
#include "leaf_sensor.h"
#include "soil_sensor.h"
#include <stdint.h>
#include <string.h>

extern aggregator_t g_aggregator;
extern volatile uint8_t g_new_sample_ready;
extern cj702_data_t g_last_sample;

static uint8_t g_tx_buf[48];

/* ── debug globals — readable via OpenOCD ── */

/* leaf sensor */
volatile uint8_t  g_dbg_leaf_ok     = 0;   /* 1=last read succeeded */
volatile int16_t  g_dbg_leaf_temp   = 0;   /* 0.01 °C */
volatile uint16_t g_dbg_leaf_hum    = 0;   /* 0.01 %RH */

/* soil sensor */
volatile uint8_t  g_dbg_soil_ok     = 0;   /* 1=last read succeeded */
volatile int16_t  g_dbg_soil_temp   = 0;   /* 0.01 °C */
volatile uint16_t g_dbg_soil_hum    = 0;   /* 0.01 %RH */
volatile uint16_t g_dbg_soil_ec     = 0;   /* us/cm */
volatile uint16_t g_dbg_soil_salt   = 0;   /* ppm */
volatile uint16_t g_dbg_soil_n      = 0;   /* mg/kg */
volatile uint16_t g_dbg_soil_p      = 0;   /* mg/kg */
volatile uint16_t g_dbg_soil_k      = 0;   /* mg/kg */
volatile uint16_t g_dbg_soil_ph     = 0;   /* pH * 10 */

/* last full soil data (extra fields not in cj702_data_t) */
volatile soil_data_t g_last_soil_data = {0};

/* soil address probing */
volatile uint8_t g_dbg_soil_addr = 0;      /* last tried address */
volatile uint8_t g_dbg_soil_addr_ok = 0;   /* address that worked (0=none yet) */

void app_fsm_init(app_fsm_t *fsm)
{
    fsm->state = FSM_ACCUMULATE;
    fsm->seconds = 0;
    fsm->no_valid_frame_seconds = 0;
    fsm->one_second_elapsed = 0;
}

void app_fsm_on_tick(app_fsm_t *fsm)
{
    fsm->one_second_elapsed = 1;
}

void app_fsm_run(app_fsm_t *fsm)
{
    if (!fsm->one_second_elapsed) {
        return;
    }
    fsm->one_second_elapsed = 0;

    if (g_new_sample_ready) {
        g_new_sample_ready = 0;
        fsm->no_valid_frame_seconds = 0;

        /* Make a local copy to avoid races with UART RX ISR */
        cj702_data_t sample = g_last_sample;

        /* ── Enrich with leaf surface sensor (RS485 ModBus, addr 0x01) ── */
        if (leaf_sensor_read(&sample.leaf_temp, &sample.leaf_wetness)) {
            g_dbg_leaf_ok   = 1;
            g_dbg_leaf_temp = sample.leaf_temp;
            g_dbg_leaf_hum  = sample.leaf_wetness;
        } else {
            g_dbg_leaf_ok   = 0;
        }

        /* ── Enrich with soil sensor (UART4 TTL ModBus) ── */
        {
            soil_data_t soil;
            bool soil_read_ok = false;

            /* Probe: if we haven't found the right address yet,
             * cycle through 0x01, 0x02, 0x03 on each attempt */
            if (g_dbg_soil_addr_ok == 0) {
                static uint8_t probe_addr = 0x01;
                g_dbg_soil_addr = probe_addr;
                if (soil_sensor_read_addr(probe_addr, &soil)) {
                    g_dbg_soil_addr_ok = probe_addr;  /* found it! */
                    soil_read_ok = true;
                } else {
                    /* try next address */
                    probe_addr++;
                    if (probe_addr > 0x03) probe_addr = 0x01;
                }
            } else {
                /* use known-good address, respect return value */
                soil_read_ok = soil_sensor_read_addr(g_dbg_soil_addr_ok, &soil);
            }

            if (soil_read_ok) {
                g_dbg_soil_ok   = 1;
                g_dbg_soil_temp = soil.temp;
                g_dbg_soil_hum  = soil.humidity;
                g_dbg_soil_ec   = soil.ec;
                g_dbg_soil_salt = soil.salt;
                g_dbg_soil_n    = soil.n;
                g_dbg_soil_p    = soil.p;
                g_dbg_soil_k    = soil.k;
                g_dbg_soil_ph   = soil.ph;
                g_last_soil_data = soil;

                sample.soil_temp     = soil.temp;
                sample.soil_humidity = soil.humidity;
                sample.ec            = soil.ec;
            } else {
                g_dbg_soil_ok = 0;
            }
        }

        /* Combined LED feedback:
         *    both ok → blue 50ms
         *    any fail → red 50ms
         */
        if (g_dbg_leaf_ok && g_dbg_soil_ok) {
            status_led_set(LED_STATE_TX);
        } else {
            status_led_set(LED_STATE_ERROR);
        }
        HAL_Delay(50);
        status_led_set(LED_STATE_NORMAL);

        aggregator_add(&g_aggregator, &sample);
    } else {
        fsm->no_valid_frame_seconds++;
    }

    fsm->seconds++;
    if (fsm->seconds < FSM_INTERVAL_SEC) {
        return;
    }
    fsm->seconds = 0;

    int len = -1;
    if (fsm->no_valid_frame_seconds >= FSM_INTERVAL_SEC) {
        status_led_set(LED_STATE_ERROR);
        len = lora_frame_pack_error(0x01, LORA_ERROR_TIMEOUT, g_tx_buf, sizeof(g_tx_buf));
    } else {
        cj702_data_t avg;
        if (aggregator_get_average(&g_aggregator, &avg)) {
            status_led_set(LED_STATE_TX);
            len = lora_frame_pack_data(0x01, &avg, g_tx_buf, sizeof(g_tx_buf));
        } else {
            status_led_set(LED_STATE_ERROR);
            len = lora_frame_pack_error(0x01, LORA_ERROR_INCOMPLETE, g_tx_buf, sizeof(g_tx_buf));
        }
    }

    if (len > 0) {
        lora_tx_send(g_tx_buf, (uint16_t)len, 1000);
    }

    aggregator_init(&g_aggregator);
    status_led_set(LED_STATE_NORMAL);
}
