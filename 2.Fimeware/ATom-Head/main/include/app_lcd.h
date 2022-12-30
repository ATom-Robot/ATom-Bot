#pragma once

#include <stdio.h>
#include <string.h>

#include "st7789.h"
#include "freertos/queue.h"

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 135
#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX / 4)

#ifdef __cplusplus
extern "C"
{
#endif

void AppLCD_Init(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb);
void AppLCD_run(void);
void AppLVGL_run(void);

#ifdef __cplusplus
}
#endif
