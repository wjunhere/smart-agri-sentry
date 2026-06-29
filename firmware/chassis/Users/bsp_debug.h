/**
 * bsp_debug.h
 * USART1 text command interface for standalone STM32 debugging.
 *
 * Connect ST-Link USB → STM32 board.
 * Open any serial terminal (Putty, Keil Serial Window, etc.) at 115200-8-N-1.
 * Type commands followed by Enter.
 */
#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H
#include "stm32f4xx_hal.h"

void Debug_Init(void);
void Debug_Process(void);

#endif
