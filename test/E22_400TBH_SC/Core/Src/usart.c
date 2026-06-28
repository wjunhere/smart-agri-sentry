/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include "fifo.h"
#include "usbd_cdc_if.h"

uint8_t usb_rx_data;
uart_feature current_feature = FUNC_FEATURE1;
static uint8_t tmp;
static uint8_t fifo_buffer[1024];
static fifo_t fifo_uart1_rx;
static volatile uint32_t uart1_rx_timeout = 0;
static bool uart1_rx_done = false;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void uart1_reconfig( uint32_t rate )
{
	/* ԭ�д���1�ĳ�ʼ�� */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = rate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* ����E22���ڽ��ն���
     �Ƽ�ʹ�ö���+��ʱ�ϰ���ʽ������E22ģ�����ݣ���Ϊ��ĳЩ�����ģ�鴮��������ݲ�һ���������ģ�����	*/
	if( fifo_create( &fifo_uart1_rx, fifo_buffer , sizeof(fifo_buffer)) != FIFO_OK )
	{
			///@todo 
			/* ���д���ʧ�� */
			while(1);
	}	
	
  /* ����E22�����жϽ��� (���ֽ��ж�) */
	HAL_UART_Receive_IT( &huart1, &tmp, 1);
	
	uart1_rx_timeout = 0;
	uart1_rx_done = false;
}

void uart1_rx_timeout_1ms_callback(void)
{
	/* ���ڽ����ж��л᲻ͣˢ�µ���ʱ
     ���δ�ʱÿ1�����ж��ڻ�ݼ�����ʱ	*/
	if( uart1_rx_timeout > 0 )
	{
			uart1_rx_timeout--;
		
			/*�������ʱ��0������Զϰ������� */
			if( uart1_rx_timeout == 0 )
			{
				uart1_rx_done = true;
			}
	}
}

void uart1_wait_response_blocked( uint8_t * buffer, uint16_t *length )
{
	uint32_t fifo_rx_len = 0;
	
	/* ������ڽ��ն��� */
	fifo_clear( &fifo_uart1_rx );
	
	/* �趨100ms����ʱ �δ�ʱ��1ms�ж��ڲ��ϵݼ�*/
	systick_set_user_timeout(100);
	
	/* �ȴ����ڽ������ */
	while( uart1_rx_done == false )
	{
			/* ���100ms����ʱ������ */
			if( systick_get_user_timeout() == 0)
			{
					///@todo
					/* ��ʱ�� ģ����Ӧ�� */
				  usb_printf( "Response Timeout!!! \r\n");
					while(1);
			}
	}	
	
	/* ������Ӧ������ */
	if( uart1_rx_done == true )
	{
		uart1_rx_done = false;

		/* ��ȡ��ǰ���ڽ��ն������ݳ��� */
		fifo_get_length( &fifo_uart1_rx , &fifo_rx_len);

		/* �������ݿ��� */
		fifo_read( &fifo_uart1_rx, buffer, fifo_rx_len );	

		*length = fifo_rx_len;
	}
}

bool uart1_check_rx_done( uint8_t *buffer , uint32_t *length )
{
	bool ret = false;
	uint32_t fifo_rx_len = 0;
	
	if( uart1_rx_done == true )
	{
		uart1_rx_done = false;
		
		/* ��ȡ��ǰ���ڽ��ն������ݳ��� */
		fifo_get_length( &fifo_uart1_rx , &fifo_rx_len);

		/* �������ݿ��� */
		fifo_read( &fifo_uart1_rx, buffer, fifo_rx_len );	

		*length = fifo_rx_len;	

		ret = true;
	}
	
	return ret;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t rd,rd_2;
	if (huart->Instance == USART1)
	{
		switch (current_feature)
		{
			case FUNC_FEATURE1:
				rd_2 = huart->Instance->DR;
				fifo_write(&fifo_uart1_rx,&rd_2,1);
				/* CDC_Transmit_FS moved to usart_fifo_flush_to_cdc() */
				HAL_UART_Receive_IT( &huart1, &usb_rx_data, 1);
				uart1_rx_timeout = 10;
                break;
			case FUNC_FEATURE2:
				rd = huart->Instance->DR;
				fifo_write(&fifo_uart1_rx,&rd,1);
                HAL_UART_Receive_IT( &huart1, &tmp, 1);
				uart1_rx_timeout = 10;
                break;
            default:
                break;
		}
	}
}
void usb_receive_to_tx_send( void )
{
	  	HAL_Delay(1);
    if(usb_rx_complete)
    {
        HAL_UART_Transmit(&huart1,my_usb_rx_data, my_usb_rx_num ,500);
        memset(my_usb_rx_data, 0, my_usb_rx_num);
        my_usb_rx_num = 0;
		usb_rx_complete = 0;
    }
}

void usart_fifo_flush_to_cdc(void)
{
    uint32_t len;
    uint8_t buf[64];
    fifo_get_length(&fifo_uart1_rx, &len);
    while (len > 0) {
        uint32_t chunk = len > sizeof(buf) ? sizeof(buf) : len;
        fifo_read(&fifo_uart1_rx, buf, chunk);
        CDC_Transmit_FS(buf, chunk);
        fifo_get_length(&fifo_uart1_rx, &len);
    }
}
/* USER CODE END 1 */
