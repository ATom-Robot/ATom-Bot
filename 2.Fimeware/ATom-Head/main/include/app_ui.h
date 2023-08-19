#pragma once

#include <lvgl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

LV_IMG_DECLARE(normal_gif);
LV_IMG_DECLARE(normal2_gif);
LV_IMG_DECLARE(listen_gif);
LV_IMG_DECLARE(listen2normal_gif);
LV_IMG_DECLARE(wakeup_gif);
LV_IMG_DECLARE(happy_gif);
LV_IMG_DECLARE(hurt_gif);
LV_IMG_DECLARE(angry_gif);

enum
{
    NORMAL_EMOJI,
    NORMAL2_EMOJI,
    LISTEN_EMOJI,
    LISTEN2NORMAL_EMOJI,
    WAKEUP_EMOJI,
    HAPPY_EMOJI,
    SAD_EMOJI,
    HURT_EMOJI,
    ANGRY_EMOJI,
};
typedef struct
{
    const void *gif;
    uint8_t repeat;
    char *voice;
} emoji_list;

extern QueueHandle_t xQueueFrameI;
extern QueueHandle_t xQueueFrameO;

void lv_emoji_create(void);

void ui_wakeup_emoji_start(void);
void ui_wakeup_emoji_over(void);

void ui_acquire(void);
void ui_release(void);

#ifdef __cplusplus
}
#endif
