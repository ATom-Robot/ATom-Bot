#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "lvgl.h"
#include "Configs/Config.h"
#include "Display_conf.hpp"

typedef LGFX_Emma SCREEN_CLASS;

void Port_Init();
void DisplayFault_Init(SCREEN_CLASS* scr);
void lv_port_disp_init(SCREEN_CLASS* scr);

extern TaskHandle_t handleTaskLvgl;
extern SCREEN_CLASS screen;

#endif