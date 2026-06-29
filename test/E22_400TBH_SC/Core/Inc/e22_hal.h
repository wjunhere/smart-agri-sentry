#ifndef _E22_HAL_H_
#define _E22_HAL_H_

#define E22_USE_GPIO_AUX  1 // 0:不使用或不检测AUX状态引脚   1:正常使用AUX引脚

#include <stdint.h>

typedef enum
{
	WORK_MODE_TRANSPARENT   = 0x00,
	WORK_MODE_WAKE_ON_RADIO = 0x01,
	WORK_MODE_CONFIG        = 0x02,
	WORK_MODE_SLEEP         = 0x03,
}work_mode_t;

void e22_hal_uart_tx( uint8_t *buffer , uint16_t length );
void e22_hal_aux_wait(void);
void e22_hal_reset(void);
void e22_hal_work_mode( work_mode_t mode);

#endif
