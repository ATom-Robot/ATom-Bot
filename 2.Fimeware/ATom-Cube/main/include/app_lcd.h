#pragma once

#include <stdio.h>
#include "Display_conf.hpp"
#include "esp_err.h"

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 240
#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX)

typedef LGFX_Emma SCREEN_CLASS;
extern SCREEN_CLASS screen;

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t AppLCD_Init(void);
esp_err_t AppLVGL_run(void);

#ifdef __cplusplus
}
#endif
