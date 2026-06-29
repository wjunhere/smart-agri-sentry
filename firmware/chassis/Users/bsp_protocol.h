/* bsp_protocol.h */
#ifndef __BSP_PROTOCOL_H
#define __BSP_PROTOCOL_H
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define RX_BUFF_SIZE 64
extern uint8_t rx_buff[RX_BUFF_SIZE];
extern volatile uint8_t rx_flag;
extern volatile uint16_t rx_len;

// �ⲿ���Ʊ���
extern volatile float target_speed_left;
extern volatile float target_speed_right;

void Protocol_Init(void);
void Protocol_Process(void);
void Protocol_Send_Telemetry(int32_t enc_left, int32_t enc_right);
#endif

