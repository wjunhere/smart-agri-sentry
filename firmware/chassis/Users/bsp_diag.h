/**
 * bsp_diag.h — Ultra-minimal boot diagnostics (NO HAL dependency).
 */
#ifndef __BSP_DIAG_H
#define __BSP_DIAG_H
#include "stm32f4xx_hal.h"

void Diag_Heartbeat_Init(void);
void Diag_Heartbeat_Toggle(void);

/* Call ONE of these as the FIRST line in main() to diagnose boot failure */
void Diag_BootBleep(void);     /* Blink PA8 forever via direct register access */
void Diag_USART1_Bleep(void);  /* Send 0x55 ('U') on USART1 at 9600 baud forever */

#endif
