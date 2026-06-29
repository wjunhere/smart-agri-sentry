/* bsp_adc.h */
#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
void BSP_ADC_Init(void);
float BSP_Get_Battery_Voltage(void);
#endif

