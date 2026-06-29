#ifndef _APPLICATION_H
#define _APPLICATION_H

#include "menu.h"
#include "menuConfig.h"

typedef struct
{
	int work_mode;
	int rate_mode;
	int channel;
	int tx_power;
	int tx_count;
}menu_config_t;

extern menu_config_t user_config;

void logo_callback( xpItem item );

void work_mode_callback( xpItem item );
void rate_mode_callback( xpItem item );
void channel_callback( xpItem item );
void tx_power_callback( xpItem item);
void tx_count_callback( xpItem item);
void background_color_callback( xpItem item );

void tx_mode_callback( xpItem item );
void rx_mode_callback( xpItem item );
void version_callback( xpItem item );
void reset_callback( xpItem item );

#endif
