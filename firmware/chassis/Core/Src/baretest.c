/* baretest.c — absolute minimum USART1 test, called from main() */
#include "stm32f4xx_hal.h"

void BareTest_USART1(void) {
    volatile uint32_t i;

    /* Enable GPIOA + USART1 clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DSB();

    /* PA9: AF push-pull, AF7 (USART1_TX) */
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODER9) | GPIO_MODER_MODER9_1;
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << 4)) | (7U << 4);
    GPIOA->OSPEEDR |= (3U << 18);

    /* USART1: 9600 baud @ HSI 16MHz, 8N1 */
    USART1->BRR  = 16000000 / 9600;
    USART1->CR1  = USART_CR1_UE | USART_CR1_TE;  /* 8N1, TX only */
    USART1->CR2  = 0;
    USART1->CR3  = 0;

    /* Send 'U' forever */
    while (1) {
        while (!(USART1->SR & USART_SR_TXE)) { __NOP(); }
        USART1->DR = 'U';
        for (i = 0; i < 200000; i++) { __NOP(); }  /* delay */
    }
}
