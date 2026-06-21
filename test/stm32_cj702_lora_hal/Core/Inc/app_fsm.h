#ifndef __APP_FSM_H
#define __APP_FSM_H

#include <stdint.h>
#include <stdbool.h>

#define FSM_INTERVAL_SEC 60

typedef enum {
    FSM_ACCUMULATE = 0,
    FSM_TX,
    FSM_WAIT,
} fsm_state_t;

typedef struct {
    fsm_state_t state;
    uint8_t seconds;
    uint8_t no_valid_frame_seconds;
    bool one_second_elapsed;
} app_fsm_t;

void app_fsm_init(app_fsm_t *fsm);
void app_fsm_on_tick(app_fsm_t *fsm);
void app_fsm_run(app_fsm_t *fsm);

#endif
