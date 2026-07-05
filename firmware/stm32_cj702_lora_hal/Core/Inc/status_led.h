#ifndef __STATUS_LED_H
#define __STATUS_LED_H

#include <stdint.h>

typedef enum {
    LED_STATE_NORMAL = 0,
    LED_STATE_TX,
    LED_STATE_ERROR,
} led_state_t;

void status_led_init(void);
void status_led_set(led_state_t state);

#endif
