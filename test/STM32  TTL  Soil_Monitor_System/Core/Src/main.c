/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Read_Sensor_Cmd[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x3F};
uint8_t Rx_Buffer[35]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
uint16_t Calculate_CRC16(uint8_t *buf, uint8_t len);
void Process_Sensor_Data(uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief printf 重定向到 UART1
  */
int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief Modbus CRC16 计算
  */
uint16_t Calculate_CRC16(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief 解析传感器数据并打印
  */
void Process_Sensor_Data(uint8_t *data, uint16_t length) {
    if (length < 21) return;

    // 校验接收到的 CRC (21字节)
    uint16_t expected_crc = Calculate_CRC16(data, 19);
    uint16_t received_crc = (data[20] << 8) | data[19]; 

    if (expected_crc != received_crc) {
        printf("[CRC Error] Data corrupted.\r\n");
        return;
    }

    // 解析寄存器 (高八位 << 8 | 低八位)
    float humi = (int16_t)((data[3] << 8) | data[4]) / 10.0f;
    float temp = (int16_t)((data[5] << 8) | data[6]) / 10.0f;
    uint16_t ec = (data[7] << 8) | data[8];
    float ph   = (float)((data[9] << 8) | data[10]) / 10.0f;
    uint16_t n  = (data[11] << 8) | data[12];
    uint16_t p  = (data[13] << 8) | data[14];
    uint16_t k  = (data[15] << 8) | data[16];
    uint16_t extra = (data[17] << 8) | data[18];

    printf("\r\n>>>> Soil Sensor Data (Slave 02) <<<<\r\n");
    printf("Humidity: %.1f %% | Temp: %.1f C\r\n", humi, temp);
    printf("PH Value: %.1f    | EC:   %d us/cm\r\n", ph, ec);
    printf("Nitrogen: %d mg/kg | Phos: %d mg/kg | Potas: %d mg/kg\r\n", n, p, k);
    printf("Salinity/TDS: %d\r\n", extra);
    printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  程序主入口
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n--- Soil Monitor System Initialized ---\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    // 1. 发送请求给传感器
    HAL_UART_Transmit(&huart2, Read_Sensor_Cmd, 8, 100);

    // 2. 接收响应
    memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, Rx_Buffer, 21, 1000);

    if (status == HAL_OK) {
        Process_Sensor_Data(Rx_Buffer, 21);
    } else {
        printf("Communication Error: %d\r\n", status);
    }

    // 状态灯闪烁
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    // 建议采样间隔为 2-5 秒
    HAL_Delay(2000); 
  }
}

/**
  * @brief 系统时钟配置 (72MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO 初始化 (PC13 灯)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief 错误处理
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
