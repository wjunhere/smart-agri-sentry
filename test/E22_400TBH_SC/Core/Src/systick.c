#include "main.h"

static volatile uint32_t user_timerout_ms = 0;

void systick_set_user_timeout( uint32_t time_ms )
{
	user_timerout_ms = time_ms;
}

uint32_t systick_get_user_timeout(void)
{
	return user_timerout_ms;
}

void systick_interrupt_1ms_callback(void)
{
	uart1_rx_timeout_1ms_callback();
	
	/* ¸¨Öú³¬Ê±ÅĞ¶Ï */
	if( user_timerout_ms > 0 )
	{
			user_timerout_ms--;
	}
}
