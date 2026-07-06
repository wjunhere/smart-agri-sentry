
#include "bsp_adc.h"


uint32_t adc_dma_buf[10]; 


static float filtered_voltage = 12.0f; 

void BSP_ADC_Init(void) {
    
    HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, 10);
}

float BSP_Get_Battery_Voltage(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        
        sum += (adc_dma_buf[i] & 0xFFFF); 
    }
    float avg_adc = (float)sum / 10.0f;

    
    float current_voltage = (avg_adc / 4096.0f) * 3.3f * 6.0f;
    
    
    filtered_voltage = 0.9f * filtered_voltage + 0.1f * current_voltage;
    
    return filtered_voltage;
}