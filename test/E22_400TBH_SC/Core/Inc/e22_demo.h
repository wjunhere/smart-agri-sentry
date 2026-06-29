#ifndef _E22_DEMO_H_
#define _E22_DEMO_H_

#include "e22_hal.h"

typedef enum
{
		OFF = 0x00,
		ON =  0x01,
}on_off_t;

typedef enum
{
		RADIO_RATE_2400   = 0x02,
		RADIO_RATE_4800   = 0x03,
		RADIO_RATE_9600   = 0x04,
		RADIO_RATE_19200  = 0x05,
		RADIO_RATE_38400  = 0x06,
		RADIO_RATE_62500  = 0x07,
}radio_rate_t;

typedef enum
{
		UART_8N1 = 0x00,
		UART_8O1 = 0x01,
		UART_8E1 = 0x02,
}uart_parity_t;

typedef enum
{
		UART_RATE_1200  = 0x00,
		UART_RATE_2400  = 0x01,
		UART_RATE_4800  = 0x02,
		UART_RATE_9600  = 0x03,
		UART_RATE_19200 = 0x04 ,
		UART_RATE_38400 = 0x05,
		UART_RATE_57600 = 0x06,
		UART_RATE_115200= 0x07,
}uart_rate_t;

typedef enum
{
		PACKET_SIZE_240 = 0x00,
		PACKET_SIZE_128 = 0x01,
		PACKET_SIZE_64  = 0x02,
		PACKET_SIZE_32  = 0x03,
}packet_size_t;

typedef enum
{
		WOR_PERIOD_500MS  = 0x00,
		WOR_PERIOD_1000MS = 0x01,
		WOR_PERIOD_1500MS = 0x02,
		WOR_PERIOD_2000MS = 0x03,
		WOR_PERIOD_2500MS = 0x04,
		WOR_PERIOD_3000MS = 0x05,
		WOR_PERIOD_3500MS = 0x06,
		WOR_PERIOD_4000MS = 0x07,
}wor_period_t;

typedef enum
{
		TX_POWER_DBM_30  = 0x00,
		TX_POWER_DBM_27  = 0x01,
		TX_POWER_DBM_24  = 0x02,
		TX_POWER_DBM_21  = 0x03,
}transmit_power_t;

typedef enum
{
		WOR_SLAVE  = 0x00,
		WOR_MASTER = 0x01,
}wor_role_t;

typedef struct
{
    /* ======== 用户配置寄存器 00H ======== */
    struct 
    {
        uint8_t address_h; /* 模组地址      (用户配置寄存器地址: 00H和01H) 不同地址的模组无法直接互通数据(广播地址除外); 65535为广播地址，可以群发信息 	*/	
    }register_0;
    
    /* ======== 用户配置寄存器 01H ======== */
    struct 
    {
        uint8_t address_l;
    }register_1;    
    
    /* ======== 用户配置寄存器 02H ======== */
    struct 
    {
        uint8_t network_id; /* 网络编组     (寄存器地址: 02H) 不同网络编号模组之间无法直接互通数据(广播地址除外)	*/		
    }register_2;   

    /* ======== 用户配置寄存器 03H ======== */
    union {
        uint8_t value;
        struct
        {
            radio_rate_t     radio_rate             : 3;  /* 无线空中速率 (寄存器地址: 03H Bit2-0) 空中速率越高，发送数据越快，但接收灵敏度会下降，表现为通信距离缩短*/
            uart_parity_t    uart_parity            : 2;  /* 串口校验类型 (寄存器地址: 03H Bit4-3) */
            uart_rate_t      uart_baud_rate         : 3;  /* 串口波特率   (寄存器地址: 03H Bit7-5) 配置模式(模式2)强制固定波特率为9600，其余传输模式时为用户配置串口波特率 */
        }field;
    }register_3;
    
    /* ======== 用户配置寄存器 04H ======== */
    union {
        uint8_t value;
        struct
        {
            transmit_power_t tx_power               : 2;  /* 发射功率               (寄存器地址: 04H Bit1-0) 不同功率模组的功率分档不一致，具体数值需要参考模组手册	*/
            on_off_t         software_switch        : 1;	/* 软件指令切换工作模式   (寄存器地址: 04H Bit3  ) 开启后，可以使用特定串口指令切换模组的工作模式(替代M0M1引脚) */
            uint8_t          reserve                : 2;  /* 保留 */
            on_off_t         environment_rssi       : 1;  /* (RSSI)环境信号强度使能 (寄存器地址: 04H Bit5  ) 开启后，可以使用指令读取芯片检测到的当前信号强度，影响因素较多，建议多次读取求均值	*/
            packet_size_t    packet_size            : 2; 	/* 串口数据分包           (寄存器地址: 04H Bit7-6) 一次性向模组内传入的字节数大于分包设定，将会拆包并逐包传输*/
        }field;
    }register_4;
    
    /* ======== 用户配置寄存器 05H ======== */
    struct 
    {
        uint8_t channel;  /* 信道编号      (寄存器地址: 05H) 具体载波频率与频段类型有关，载波频率 = 信道0起始频率 + (1MHz x 信道编号)  */
    }register_5;       

    /* ======== 用户配置寄存器 06H ======== */
    union {
        uint8_t value;
        struct
        {
            wor_period_t     wake_on_radio_period    : 3;  /* 空中唤醒WOR周期                  (寄存器地址: 06H Bit2-0) 收发双端设定周期最好一样，否则容易出现无法唤醒现象 */	
            wor_role_t       wake_on_radio_role      : 1;  /* 空中唤醒WOR角色                  (寄存器地址: 06H Bit3  ) 接收端(从)：会根据设定周期进行休眠与自动唤醒，降低功耗; 发送端(主)：发送数据时会根据设定周期发送足够时间的唤醒码，把接收端叫醒再传输用户数据 */	
            on_off_t         listen_before_talk      : 1;  /* 避让功能使能(Listen before talk) (寄存器地址: 06H Bit4  ) 发送时会在2秒内自动判断当前信道是否忙，并尝试发送 */	
            on_off_t         router_mode             : 1;  /* 中继模式使能                     (寄存器地址: 06H Bit5  ) 将支持转发空中数据，请阅读手册中的图文介绍了解详细用法。注意为半双工中继，且转发需要考虑延迟时间		 */
            on_off_t         specify_target          : 1;  /* 指定目标传输，也叫定点模式       (寄存器地址: 06H Bit6  ) 将用户串口传入数据的前三字节用来改变地址与信道 */
            on_off_t         packet_rssi             : 1;  /* (RSSI)接收数据包信号强度使能     (寄存器地址: 06H Bit7  ) 接收到正确无线数据并串口输出时会在其末尾附加一字节该数值 */
        }field;
    }register_6;
    
    /* ======== 用户配置寄存器 07H ======== */
    struct 
    {
        uint8_t password_h;  /* 通信密匙 (寄存器地址: 07H和08H) 密匙不一样的模组无法互通。注意，可写入不可读取，就算读取也只能得到0 */
    }register_7; 
    
    /* ======== 用户配置寄存器 08H ======== */
    struct 
    {
        uint8_t password_l;
    }register_8;     
    
    /* ======== 用户配置寄存器 09H ======== */
    struct 
    {
        uint8_t wor_rx_echo_time_h; /* 空中唤醒模式应答时间窗口 (寄存器地址: 09H和0AH) 设定时间后，如果WOR接收端被唤醒并成功接收到数据包后，将会继续保持唤醒一段时间，且该时间内允许直接透传数据，适用于被唤醒+应答场景，这样可以不用切换模式 */
    }register_9;     
    
    /* ======== 用户配置寄存器 0AH ======== */
    struct 
    {
        uint8_t wor_rx_echo_time_l;
    }register_10;   

}e22_register_t;

typedef enum
{
		REQUEST_CMD_CONFIG    =  0x00,
		REQUEST_CMD_NAME      ,
		REQUEST_CMD_VERSION   ,
		REQUEST_CMD_ENV_RSSI 
}request_cmd_t;

typedef struct
{
	uint8_t address_h;
	uint8_t address_l;
	uint8_t channel;
	uint8_t data[237];//定点传输(指定目标)单包数据不得超过237字节，否则超出部分数据会丢失
}e22_specify_target_buffer_t;

typedef struct
{
	uint8_t command;
	uint8_t register_start;
	uint8_t register_number;
	uint8_t config[20];
}e22_hex_cmd_buffer_t;

typedef union 
{
	uint8_t opt_buffer[1024];
  e22_hex_cmd_buffer_t        hex_cmd;    
	e22_specify_target_buffer_t target;
}e22_opt_buffer_t;

#endif

