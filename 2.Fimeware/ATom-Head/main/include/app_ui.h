#pragma once

#include "esp_err.h"
#include "ui.h"

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
LV_IMG_DECLARE(emo_gif);
LV_IMG_DECLARE(scared_gif);
LV_IMG_DECLARE(angry_gif);
LV_IMG_DECLARE(shake_gif);

typedef enum
{
    NORMAL_EMOJI,
    NORMAL2_EMOJI,
    LISTEN_EMOJI,
    LISTEN2NORMAL_EMOJI,
    WAKEUP_EMOJI,
    HAPPY_EMOJI,
    SING1_EMOJI,
    SING2_EMOJI,
    EMO_EMOJI,
    SCARED_EMOJI,
    ANGRY_EMOJI,
    SHAKE_EMOJI,
}emoji_t;
typedef struct
{
    const void *gif;
    uint16_t timeout;
    char *voice;
} emoji_list;

extern emoji_list em_list[];
extern const char *song_list[];

// 歌曲数量
#define SONG_COUNT 2

// gif screen
extern lv_obj_t *ui_screen_main;

void ui_emoji_create(void);
void ui_wakeup_emoji_start(void);
void ui_wakeup_emoji_over(void);
void ui_shaked_emoji_start(void);
void ui_shaked_emoji_over(void);
void ui_sr_emoji_display(emoji_t emoji, bool flag);
void ui_set_mac_address(uint8_t *address);
void ui_set_ip_address(const char *address);
void ui_set_wifi_ssid(const char *ssid);
void ui_set_info_to_statusbar(const char *text);
void ui_set_wifi_icon_status(bool status);
void ui_set_joint_angle(int16_t angle);
void ui_set_menu(uint8_t *page_index);

void ui_main_screen_event(lv_event_t *e);

void ui_acquire(void);
void ui_release(void);

#ifdef __cplusplus
}
#endif
