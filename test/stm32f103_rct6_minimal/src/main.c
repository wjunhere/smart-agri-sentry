#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

#define LED_R_PIN   GPIO_Pin_0
#define LED_G_PIN   GPIO_Pin_1
#define LED_B_PIN   GPIO_Pin_2
#define LED_PORT    GPIOC
#define LED_CLK     RCC_APB2Periph_GPIOC

#define KEY_PIN     GPIO_Pin_5
#define KEY_PORT    GPIOB
#define KEY_CLK     RCC_APB2Periph_GPIOB

static void delay_ms(uint32_t ms)
{
    volatile uint32_t n;
    while (ms--) {
        for (n = 8000; n > 0; n--)
            __asm__("nop");
    }
}

static void rgb_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);

    gpio.GPIO_Pin = LED_R_PIN | LED_G_PIN | LED_B_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_PORT, &gpio);

    GPIO_SetBits(LED_PORT, LED_R_PIN | LED_G_PIN | LED_B_PIN);
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (r) GPIO_ResetBits(LED_PORT, LED_R_PIN); else GPIO_SetBits(LED_PORT, LED_R_PIN);
    if (g) GPIO_ResetBits(LED_PORT, LED_G_PIN); else GPIO_SetBits(LED_PORT, LED_G_PIN);
    if (b) GPIO_ResetBits(LED_PORT, LED_B_PIN); else GPIO_SetBits(LED_PORT, LED_B_PIN);
}

static void key_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(KEY_CLK, ENABLE);

    gpio.GPIO_Pin = KEY_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY_PORT, &gpio);
}

static uint8_t key_is_pressed(void)
{
    return GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == Bit_RESET;
}

static void usart1_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);
}

int fputc(int ch, FILE *f)
{
    (void)f;
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
    return ch;
}

int main(void)
{
    uint32_t counter = 0;
    const uint8_t colors[][3] = {
        {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
        {1, 1, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 1}
    };
    const char *color_names[] = {
        "red", "green", "blue",
        "yellow", "cyan", "magenta", "white"
    };
    uint8_t idx = 0;

    rgb_init();
    key_init();
    usart1_init(115200);

    printf("\r\n=== STM32F103RCT6 minimal test started ===\r\n");
    printf("Board: YB-MSA02-v1.1, HSE 8MHz, SYSCLK 72MHz\r\n");

    while (1) {
        rgb_set(colors[idx][0], colors[idx][1], colors[idx][2]);
        printf("[%lu] LED: %s, KEY1: %s\r\n",
               (unsigned long)counter,
               color_names[idx],
               key_is_pressed() ? "PRESSED" : "released");

        delay_ms(500);

        if (key_is_pressed()) {
            delay_ms(50);
            if (key_is_pressed()) {
                printf("KEY1 pressed, skip color change\r\n");
                while (key_is_pressed())
                    delay_ms(10);
                continue;
            }
        }

        idx = (idx + 1) % 7;
        counter++;
    }
}

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1)
        ;
}
