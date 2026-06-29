#include "u8g2_hal.h"
#include "menuConfig.h"
#include "i2c.h"


#define HARDWARE_I2C
#define OLED_I2C_DEV_ADDRESS  0x78 


#ifdef HARDWARE_I2C
static uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    uint8_t* data = (uint8_t*) arg_ptr;
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            while( arg_int-- > 0 ) {

								///@todo 修改
								/* ==================================== */
								I2C2->DR = *data++;
								while( __HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_TXE) == RESET );
								/* ==================================== */
            }
            break;
        case U8X8_MSG_BYTE_INIT:
        /* add your custom code to init i2c subsystem */
            break;
        case U8X8_MSG_BYTE_SET_DC:
        /* ignored for i2c */
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
					
						///@todo 修改
						/* ==================================== */
						/* Wait until BUSY flag is reset */
            while( __HAL_I2C_GET_FLAG( &hi2c2, I2C_FLAG_BUSY ) == SET );
						/* Disable Pos */
						CLEAR_BIT( I2C2->CR1, I2C_CR1_POS );				
						/* Generate Start */
						SET_BIT( I2C2->CR1, I2C_CR1_START );
						/* Wait until SB flag is set */
						while( __HAL_I2C_GET_FLAG( &hi2c2, I2C_FLAG_SB ) == RESET );
						/* Send slave address */
						I2C2->DR = I2C_7BIT_ADD_WRITE( OLED_I2C_DEV_ADDRESS );				
						/* Wait until ADDR flag is set */
						while ( __HAL_I2C_GET_FLAG( &hi2c2, I2C_FLAG_ADDR) == RESET );
						/* Clear ADDR flag */
						__HAL_I2C_CLEAR_ADDRFLAG( &hi2c2 );				
						/* Wait until TXE flag is set */
				    while( __HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_TXE) == RESET );
						/* ==================================== */
						
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
					
						///@todo 修改
						/* ==================================== */            
						/* Generate Stop */
						SET_BIT( I2C2->CR1, I2C_CR1_STOP );
						/* ==================================== */
				
            break;
        default:
            return 0;
    }
    return 1;
}
static uint8_t u8x8_gpio_and_delay_hw(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
            break;
        case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
            break;
        case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
            HAL_Delay(1);
            break;
        case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
        case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
            break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
        case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
            break;                    // arg_int=1: Input dir with pullup high for I2C data pin
        case U8X8_MSG_GPIO_MENU_SELECT:
            u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_NEXT:
            u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_PREV:
            u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_HOME:
            u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;
    }
    return 1;
}

//static void HardWare_I2C2_GPIOInit(void)
//{

//}

#endif 



void u8g2Init(u8g2_t *u8g2)
{

	#ifdef HARDWARE_I2C
//    HardWare_I2C2_GPIOInit();//已在i2c.c内完成初始化
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay_hw);
  #endif 
     
	u8g2_InitDisplay(u8g2);                                                                       
	u8g2_SetPowerSave(u8g2, 0);                                                                   
	u8g2_ClearBuffer(u8g2);
}
