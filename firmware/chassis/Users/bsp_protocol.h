/* bsp_protocol.h */
#ifndef __BSP_PROTOCOL_H
#define __BSP_PROTOCOL_H
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define FRAME_HEADER_0 0xAA
#define FRAME_HEADER_1 0x55
#define TYPE_CHASSIS    0x03
#define TYPE_MOTION_CMD 0x81

#define RX_BUFF_SIZE 64
extern uint8_t rx_buff[RX_BUFF_SIZE];
extern volatile uint8_t rx_flag;
extern volatile uint16_t rx_len;

// 外部控制变量
extern volatile float target_speed_left;
extern volatile float target_speed_right;

void Protocol_Init(void);
void Protocol_Process(void);
void Protocol_Send_Chassis_Status(int16_t left_speed_mm_s, int16_t right_speed_mm_s, int16_t battery_x100, uint8_t alarm_bits, int32_t left_pulse, int32_t right_pulse, uint32_t timestamp_ms);

uint8_t Protocol_Get_CommErrorCount(void);
void Protocol_Clear_CommErrorCount(void);

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
#endif
