#pragma once

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "Display_conf.hpp"

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 134
#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX)

typedef LGFX_Emma SCREEN_CLASS;
extern SCREEN_CLASS screen;

#ifdef __cplusplus
extern "C"
{
#endif

    void AppLCD_Init(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb);
    void AppLCD_run(void);
    void AppLVGL_run(void);

    void ui_acquire(void);
    void ui_release(void);

#ifdef __cplusplus
}
#endif
