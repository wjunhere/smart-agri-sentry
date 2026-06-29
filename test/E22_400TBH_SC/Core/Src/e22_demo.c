#include "e22_demo.h"
#include "main.h"
#include "application.h"
#include <string.h>

/** 
 * 模组数据操作缓存
 */
static e22_opt_buffer_t e22_buffer;

/** 
 * 第1字节 ：控制指令  0xC1代表读取;
 * 第2字节 ：起始地址  0x00代表从寄存器地址0开始读取 
 * 第3字节 ：读取长度  0x0B代表一共读取11个寄存器参数 
 */			
static const uint8_t request_config[3]={0xC1,0x00,0x0B};

/** 
 * 第1~4字节：特殊读取指令头 C0 C1 C2 C3 
 * 第5字节  ：起始地址  00 			
 * 第6字节  ：读取长度  01 
 */				
static const uint8_t request_env_rssi[6]={0xC0,0xC1,0xC2,0xC3,0x00,0x01};

/** 
 * AT指令 读取设备名
 */
static const char request_name[]="AT+DEVTYPE=?";

/** 
 * AT指令 读取固件版本
 */
static const char request_version[]="AT+FWCODE=?";

/** 
 * E22-400T30S 默认配置参数
 */
static const e22_register_t register_default = 
{
	.register_0 = {
		.address_h = 0x00,
	},
	.register_1 = {
		.address_l = 0x00,
	},
	.register_2 = {
		.network_id = 0x00,
	},
	.register_3.field = {
		.radio_rate       = RADIO_RATE_2400,
		.uart_parity      = UART_8N1,
		.uart_baud_rate   = UART_RATE_9600,
	},
	.register_4.field = {
		.tx_power         = TX_POWER_DBM_30,
		.software_switch  = OFF,
		.environment_rssi = OFF,
		.packet_size      = PACKET_SIZE_240,
	},
	.register_5 = {
		.channel = 0x17,
	},          
	.register_6.field = {
		.wake_on_radio_period  = WOR_PERIOD_2000MS,
		.wake_on_radio_role    = WOR_SLAVE,
		.listen_before_talk    = OFF,
		.router_mode           = OFF,
		.specify_target        = OFF,
		.packet_rssi           = OFF,
	},
	.register_7 = {
		.password_h = 0x00,
	},  
	.register_8 = {
		.password_l = 0x00,
	},            
	.register_9 = {
		.wor_rx_echo_time_h = 0x00,
	},
	.register_10 = {
		.wor_rx_echo_time_l = 0x00,
	},  
};

/**
 * @brief  向模组写入配置参数指令
 *
 * @param  config 配置信息
 */
static void e22_send_config_command( const e22_register_t *config )
{
	/* 第1字节：控制指令 
	   C0代表写入参数并保存 */
	e22_buffer.hex_cmd.command = 0xC0;
	
	/* 第2字节：起始地址
	   00代表从寄存器地址0开始写入 */
	e22_buffer.hex_cmd.register_start = 0x00;
	
	/* 第3字节：写入数量 */	
	e22_buffer.hex_cmd.register_number = sizeof(e22_register_t);
	
	/* 第4字节开始就是用户配置参数了 */
	memcpy( e22_buffer.hex_cmd.config , (uint8_t*)config , sizeof(e22_register_t));
	
	/* 串口写入 */
	e22_hal_uart_tx( (uint8_t*)&e22_buffer, sizeof(e22_register_t) + 3);
}

/**
 * @brief  向模组写入查询指令
 *
 * @param  cmd 查询类型
 */
static void e22_send_request_command( request_cmd_t cmd )
{
	switch( cmd )
	{
		/* 读取配置参数 */
		case REQUEST_CMD_CONFIG:
			e22_hal_uart_tx( (uint8_t*)request_config, sizeof(request_config));		
			break;
		
		/* 读取设备名称 */
		case REQUEST_CMD_NAME:
			e22_hal_uart_tx( (uint8_t*)request_name, strlen(request_name));	
			break;

		/* 读取固件版本 */
		case REQUEST_CMD_VERSION:
			e22_hal_uart_tx( (uint8_t*)request_version, strlen(request_version));	
			break;		
		/* 读取环境RSSI */		
		case REQUEST_CMD_ENV_RSSI:
			e22_hal_uart_tx( (uint8_t*)request_env_rssi, sizeof(request_env_rssi));				
			break;
	}
}

/**
 * @brief  (查询指令的)模组应答数据检查
 *
 * @param   cmd 查询类型
 * @param   buffer 指向应答数据缓存
 * @param   length 应答数据长度
 * @return  bool 正确返回true; 否则返回false。
 */
static bool e22_response_command_check( request_cmd_t cmd , uint8_t *buffer , uint8_t length )
{
	bool ret = false;
	
	switch( cmd )
	{
		/* 读取配置参数 */	
		case REQUEST_CMD_CONFIG:
			
		  /* 长度检查 必须等于查询指令指定的寄存器数量+3字节帧头*/
			if( length == (request_config[2] + 3))
			{
				/* 内容检查 帧头必须与查询指令一致 */
				if( buffer[0] == request_config[0] && buffer[1] == request_config[1] && buffer[2] == request_config[2])			
				{
						ret = true;
				}
			}
			break;
		
		/* 读取设备名称 */	
		case REQUEST_CMD_NAME:
			/* 内容检查 */
			if( strncmp( "DEVTYPE=", (char*)buffer, 8) == 0 )
			{
					ret = true;
			}	
			break;
		
		/* 读取固件版本 */
		case REQUEST_CMD_VERSION:
			/* 内容检查 */
			if( strncmp( "FWCODE=", (char*)buffer, 7) == 0 )
			{
					ret = true;
			}	
			break;		
			
		/* 读取环境RSSI */	
		case REQUEST_CMD_ENV_RSSI:
		  /* 长度检查 固定返回4字节*/
			if( length == 4)
			{
				/* 内容检查 固定格式 C1 00 01 + RSSI */
				if( buffer[0] == 0xC1 && buffer[1] == 0x00 && buffer[2] == 0x01)
				{
						ret = true;
				}	
			}
			break;			
	}
	
	return ret;
}

/**
 * @brief  向模组串口写入数据
 *
 * @note 请注意模组工作模式
 * @param buffer 指向数据缓存
 * @param length 写入长度
 */
void e22_demo_transmit( uint8_t *buffer , uint16_t length )
{
	/* 串口写入 */
	e22_hal_uart_tx( buffer, length);	
	
}


/**
 * @brief  (可选) 示例如何读取设备名称
 *
 * @note 模组应处于配置模式
 * @param buffer 模组应答内容
 * @param length 模组应答长度
 */
void e22_demo_read_device_name( char *buffer , uint8_t *length  )
{
	uint16_t opt_length;

	/* 配置模式仅支持串口9600波特率 */
	uart1_reconfig(9600);
	
	/* 切换到配置模式 */
	e22_hal_work_mode( WORK_MODE_CONFIG );
	
	/* 发送查询指令 获取设备名 */
	e22_send_request_command( REQUEST_CMD_NAME );	
	
	/* 等待模组应答 */
	uart1_wait_response_blocked( (uint8_t*)buffer , &opt_length );
	
	/* 应答内容检查 */
	if( e22_response_command_check( REQUEST_CMD_NAME, (uint8_t*)buffer , opt_length ) != true )
	{
			/* 应答数据不正确 */
			while(1);	
	}	

	*length = opt_length;
}

/**
 * @brief  (可选) 示例如何读取设备固件版本
 *
 * @note 模组应处于配置模式
 * @param buffer 模组应答内容
 * @param length 模组应答长度
 */
void e22_demo_read_fireware_version( char *buffer , uint8_t *length )
{
	uint16_t opt_length;
	
	/* 配置模式仅支持串口9600波特率 */
	uart1_reconfig(9600);	
	
	/* 切换到配置模式 */
	e22_hal_work_mode( WORK_MODE_CONFIG );	
	
	/* 发送查询指令 获取设备名 */
	e22_send_request_command( REQUEST_CMD_VERSION );	
	
	/* 等待模组应答 */
	uart1_wait_response_blocked( (uint8_t*)buffer , &opt_length );
	
	/* 应答内容检查 */
	if( e22_response_command_check( REQUEST_CMD_VERSION, (uint8_t*)buffer , opt_length ) != true )
	{
			/* 应答数据不正确 */
			while(1);			
	}
			
	*length = opt_length;
}

/**
 * @brief  (可选) 示例如何读取设备环境RSSI
 *
 * @note 模组应处于透明传输模式
 * @param buffer 模组应答内容
 * @param length 模组应答长度
 */
void e22_demo_env_rssi_read( int8_t *rssi )
{
	uint16_t opt_length;

	/* 发送查询指令 获取设备名 */
	e22_send_request_command( REQUEST_CMD_ENV_RSSI );	
	
	/* 等待模组应答 */
	uart1_wait_response_blocked( e22_buffer.opt_buffer , &opt_length );

	/* 应答内容检查 */
	if( e22_response_command_check( REQUEST_CMD_ENV_RSSI, e22_buffer.opt_buffer , opt_length ) != true )
	{
			/* 读取超时 或 应答数据不正确 */
			while(1);			
	}
			
	*rssi = (int8_t)e22_buffer.opt_buffer[3];
}

/**
 * @brief  使用显示菜单用户配置参数重新配置模组
 *
 * @param config 菜单配置参数信息
 */
void e22_demo_menu_config( menu_config_t *config )
{
	uint16_t opt_length;
	e22_register_t register_write;
	transmit_power_t power_select;
	
	/* 复制默认寄存器配置 */
	memcpy( (uint8_t*)&register_write , (uint8_t*)&register_default ,  sizeof(e22_register_t) );
	/* 功率转换 */	
	switch( config->tx_power )
	{
		case 27: 
			power_select = TX_POWER_DBM_27; 
			break;
		
		case 24:
			power_select = TX_POWER_DBM_24; 
			break;			
		
		case 21:
			power_select = TX_POWER_DBM_21; 
			break;	

		default:
			power_select = TX_POWER_DBM_30; 
			break;
	}
	
	/* 修改用户配置的参数 */
	register_write.register_4.field.tx_power = power_select;                      //功率	
	register_write.register_3.field.radio_rate = (radio_rate_t)config->rate_mode; //空速
	register_write.register_5.channel = config->channel;                          //信道
	
	/* 开启数据包RSSI RX测试需要使用 */
	register_write.register_6.field.packet_rssi = ON;   
	
	/* 配置模式仅支持串口9600波特率 */
	uart1_reconfig(9600);		
	
	/* 切换到配置模式 */
	e22_hal_work_mode( WORK_MODE_CONFIG );
	
	/* 向模组串口写入配置 */
	e22_send_config_command( &register_write );	
	
	/* 等待模组应答 */
	uart1_wait_response_blocked( e22_buffer.opt_buffer , &opt_length );
	
	/* 应答数据检查 指令头必为C1 */
	if( e22_buffer.opt_buffer[0] != 0xC1 )
	{
			/* 模组如果回答FF FF FF 表示指令错误 */
			while(1);
	}
	
	///@todo 请结合串口波特率配置修改
	/* 切换到用户设置的透传模式串口波特率 */
	uart1_reconfig(9600);			
	
	/* 回到透明传输模式 */
	e22_hal_work_mode( WORK_MODE_TRANSPARENT );	
}

