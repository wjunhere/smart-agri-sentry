#include "app_fsm.h"
#include "aggregator.h"
#include "lora_frame.h"
#include "lora_tx.h"
#include "status_led.h"
#include "cj702.h"
#include <stdint.h>
#include <string.h>

extern aggregator_t g_aggregator;
extern volatile uint8_t g_new_sample_ready;
extern cj702_data_t g_last_sample;

static uint8_t g_tx_buf[32];

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
        aggregator_add(&g_aggregator, &g_last_sample);
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
