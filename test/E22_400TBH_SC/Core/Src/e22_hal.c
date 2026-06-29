#include "e22_hal.h"

/**
 * 与单片机平台有关
 */
#include "main.h"
#include "gpio.h"
#include "usart.h"

/**
 * @brief 串口发送接口
 *
 * @param buffer 发送内容
 * @param length 发送长度 
 */
void e22_hal_uart_tx( uint8_t *buffer , uint16_t length )
{
		HAL_UART_Transmit( &huart1, buffer, length, 0xFFFF);
}

/**
 * @brief 模组忙状态等待 
 */
void e22_hal_aux_wait(void)
{
	if( HAL_GPIO_ReadPin( AUX_GPIO_Port , AUX_Pin ) == GPIO_PIN_RESET )
	{
			/* 先等模块空闲 AUX引脚低为忙、高为空闲 */
			while( HAL_GPIO_ReadPin( AUX_GPIO_Port , AUX_Pin ) == GPIO_PIN_RESET )
			{
					///@todo 可以做超时逻辑
			}	
			
			/* 考虑兼容性，AUX空闲后再稍等1~2ms */
			HAL_Delay(2);	
	}
}

/**
 * @brief 模组复位 
 */
void e22_hal_reset(void)
{
	HAL_GPIO_WritePin( RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET );
	
	HAL_Delay(1);
	
	HAL_GPIO_WritePin( RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET );
	
#if E22_USE_GPIO_AUX		
	/* 注意 硬件重启时并不是立即AUX输出低电平，需要等待一段时间再开始AUX检测 */
	HAL_Delay(10);
	
	e22_hal_aux_wait();
#else
//	/* E22-V2 (V2.1) 版本启动时间较长 
//	   可以通过AT+FWCODE=?读取版本信息进行确认 7364-x-xx */
//	e22_delay_ms(500);
	
	/* E22-V2 (V2.2) 版本
		 可以通过AT+FWCODE=?读取版本信息进行确认 7453-x-xx 	*/
	e22_delay_ms(30);
#endif	
	
}

/**
 * @brief 模组模式切换
 *
 * @param mode 工作模式
 */
void e22_hal_work_mode( work_mode_t mode)
{

#if E22_USE_GPIO_AUX	
	e22_hal_aux_wait();
#endif
	
	switch(mode)
	{
		/* 模式0： 透传模式 (M0=0 M1=0) */
		case WORK_MODE_TRANSPARENT:
			HAL_GPIO_WritePin( M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET );
			break;
		
		/* 模式1： 空中唤醒模式 (M0=1 M1=0) */
		case WORK_MODE_WAKE_ON_RADIO:
			HAL_GPIO_WritePin( M0_GPIO_Port, M0_Pin, GPIO_PIN_SET );
			HAL_GPIO_WritePin( M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET );		
			break;
		
		/* 模式2： 配置模式 (M0=0 M1=1) */
		case WORK_MODE_CONFIG:
			HAL_GPIO_WritePin( M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( M1_GPIO_Port, M1_Pin, GPIO_PIN_SET );		
			break;
		
		/* 模式3： 休眠模式 (M0=1 M1=1) */
		case WORK_MODE_SLEEP:
			HAL_GPIO_WritePin( M0_GPIO_Port, M0_Pin, GPIO_PIN_SET );
			HAL_GPIO_WritePin( M1_GPIO_Port, M1_Pin, GPIO_PIN_SET );		
			break;	
		
		default: 
			while(1); //模式错误
	}
	
#if E22_USE_GPIO_AUX	
	/* 引脚切换后 模组有时AUX引脚并不是立即输出低电平，需要稍等再进行检查 */
	HAL_Delay(5);
	e22_hal_aux_wait();
#else
	
	/* 这里保守一点延时，确保模式切换已正确完成 */
	e22_delay_ms(50);
#endif	
	
}




