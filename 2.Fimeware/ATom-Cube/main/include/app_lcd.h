#pragma once

#include <stdio.h>
#include <string.h>

#include "st7789.h"

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 240
#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX / 4)

#ifdef __cplusplus
extern "C"
{
#endif

void AppLCD_Init(void);
void AppLCD_run(void);
void AppLVGL_run(void);

#ifdef __cplusplus
}
#endif
