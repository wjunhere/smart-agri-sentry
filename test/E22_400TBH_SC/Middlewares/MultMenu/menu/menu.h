#ifndef __MENU_H__
#define __MENU_H__

#include "stdbool.h"
#include "menuConfig.h"

void Draw_DialogBox(uint16_t x,uint16_t y,uint16_t w,uint16_t h);
void Draw_DialogRBox(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t r);
bool DialogScale_Show(uint8_t x,uint8_t y,uint8_t Targrt_w,uint8_t Targrt_h);
void Set_BgColor(uint8_t color);
uint8_t Get_BgColor(void);
int Draw_Scrollbar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, double min, double max, int step, int NowValue);
void Menu_Task(void);
void Menu_Init(void);

#endif
