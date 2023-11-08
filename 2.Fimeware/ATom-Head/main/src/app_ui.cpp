#include "app_ui.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "app_player.h"
// #include "benchmark/lv_demo_benchmark.h"

static const char *TAG = "UI";

/*********************
 *      DEFINES
 *********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
emoji_list em_list[] =
{
    {&normal_gif, 1, ""},
    {&normal2_gif, 1, ""},
    {&listen_gif, 0, "listen.mp3"},
    {&listen2normal_gif, 0, ""},
    {&wakeup_gif, 1, "wakeup.mp3"},
    {&happy_gif, 1, "happy.mp3"},
    {&hurt_gif, 1, "hurt.mp3"},
    {&angry_gif, 1, "angry.mp3"},
};

extern bool gReturnFB;
static lv_obj_t *gif_anim = NULL;

#if 0
extern lv_obj_t *camera_obj;

void lv_camera_create(void)
{
    camera_obj = lv_img_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);
    /*Set a background*/
    lv_obj_add_style(camera_obj, &style, 0);

    lv_obj_center(camera_obj);
    lv_obj_set_size(camera_obj, 240, 135);
}

static void camera_ui_task(void *arg)
{
    camera_fb_t *frame = NULL;

    static lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = 240;
    img_dsc.header.h = 135;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data_size = 240 * 135 * 2;
    img_dsc.data = NULL;

    while (true)
    {
        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
            // get camera farm
            img_dsc.data = frame->buf;
            lv_img_set_src(camera_obj, &img_dsc);

            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }
        }
    }
}
#endif

void next_frame_task_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    bool active = (bool) event->param;
    static bool is_wakeup = false;
    static uint8_t normal_cnt = NORMAL_EMOJI;

    switch (code)
    {
    case LV_EVENT_READY:
    {
        printf("----gif play finsh----\n");

        /* normal loop */
        if (!is_wakeup)
        {
            lv_gif_set_src(gif_anim, em_list[normal_cnt].gif);
            normal_cnt >= 1 ? normal_cnt = 0 : normal_cnt += 1;
            ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
        }

        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        /* wake up to emoji */
        ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
        active == true ? is_wakeup = true : is_wakeup = false;
        break;
    }
    default:
        break;
    }
}

static void wakeup_voice_cb(lv_timer_t *timer)
{
    /* wake up voice */
    app_player_play_name(em_list[WAKEUP_EMOJI].voice);
    lv_timer_del(timer);
}

void lv_emoji_create(void)
{
    gif_anim = lv_gif_create(lv_scr_act());
    lv_obj_add_event_cb(gif_anim, next_frame_task_cb, LV_EVENT_ALL, NULL);

    lv_gif_set_src(gif_anim, em_list[WAKEUP_EMOJI].gif);
    lv_obj_align(gif_anim, LV_ALIGN_CENTER, 0, 0);

    lv_timer_t *t = lv_timer_create(wakeup_voice_cb, 6800, NULL);
    lv_timer_set_repeat_count(t, 1);

    ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
}

void ui_wakeup_emoji_start(void)
{
    ui_acquire();

    lv_gif_set_src(gif_anim, em_list[LISTEN_EMOJI].gif);

    lv_event_send(gif_anim, LV_EVENT_VALUE_CHANGED, (void *) true);

    ui_release();
}

void ui_wakeup_emoji_over(void)
{
    ui_acquire();

    lv_gif_set_src(gif_anim, em_list[LISTEN2NORMAL_EMOJI].gif);

    lv_event_send(gif_anim, LV_EVENT_VALUE_CHANGED, (void *) false);

    ui_release();
}
