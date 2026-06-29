/**
 * bsp_diag.c — Ultra-minimal boot diagnostic.
 *
 * Diag_BootBleep():
 *   Configures PA8 as output WITHOUT HAL, toggles it in a software delay loop
 *   so you can measure PA8 with a multimeter (should alternate 0V / 3.3V).
 *
 * Diag_USART1_Bleep():
 *   Configures USART1 registers directly (no HAL), sends 0x55 ('U') continuously
 *   at 9600 baud with HSI 16MHz. Use any serial terminal at 9600-8-N-1.
 *
 * Call these BEFORE HAL_Init() if you suspect the startup code is crashing.
 */
#include "bsp_diag.h"

/* ---- Direct register access, NO HAL dependency ---- */

#define DELAY_LOOPS  2000000  /* ~500ms at 16MHz (rough) */

void Diag_Heartbeat_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __DSB();
    GPIOA->MODER   = (GPIOA->MODER   & ~GPIO_MODER_MODER8)  | GPIO_MODER_MODER8_0;
    GPIOA->OTYPER  = (GPIOA->OTYPER  & ~GPIO_OTYPER_OT_8);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << 16));       /* clear bits 16-17 (pin 8 speed = low) */
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~GPIO_PUPDR_PUPDR8);
}

void Diag_Heartbeat_Toggle(void) {
    /* Toggle PA8 via ODR */
    GPIOA->ODR ^= GPIO_ODR_OD8;
}

/* Blink PA8 with a software delay — visible with a multimeter on DC voltage
   (should read ~1.6V average if toggling at 50% duty) */
void Diag_BootBleep(void) {
    volatile uint32_t i;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __DSB();
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODER8) | GPIO_MODER_MODER8_0;
    GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_8;
    GPIOA->OSPEEDR &= ~(3U << 16);       /* pin 8 speed = low */
    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR8;

    /* Toggle forever — if you measure PA8 with multimeter:
       DC ~1.65V → MCU alive and toggling
       0V or 3.3V steady → MCU not reaching this code */
    while (1) {
        GPIOA->BSRR = GPIO_BSRR_BS8;   /* PA8 HIGH */
        for (i = 0; i < DELAY_LOOPS; i++) { __NOP(); }
        GPIOA->BSRR = GPIO_BSRR_BR8;   /* PA8 LOW */
        for (i = 0; i < DELAY_LOOPS; i++) { __NOP(); }
    }
}

/* Send 0x55 ('U') continuously on USART1 (PA9) at 9600 baud, HSI 16MHz.
   Open serial terminal at 9600-8-N-1. If you see 'U's, USART1 TX works. */
void Diag_USART1_Bleep(void) {
    volatile uint32_t i;

    /* Enable GPIOA + USART1 clocks */
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;
    __DSB();

    /* PA9: AF push-pull, AF7 (USART1) */
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODER9) | GPIO_MODER_MODER9_1;
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << 4)) | (7U << 4);  /* AFRH9 = AF7 */
    GPIOA->OSPEEDR |= (3U << 18);  /* pin 9 speed = high */

    /* USART1: 9600 baud, 8N1 */
    USART1->BRR  = 16000000 / 9600;   /* ~1667 */
    USART1->CR1  = USART_CR1_UE | USART_CR1_TE;  /* Enable, TX only */
    USART1->CR2  = 0;
    USART1->CR3  = 0;

    while (1) {
        while (!(USART1->SR & USART_SR_TXE)) { __NOP(); }
        USART1->DR = 0x55;  /* 'U' */
        for (i = 0; i < DELAY_LOOPS; i++) { __NOP(); }
    }
}
