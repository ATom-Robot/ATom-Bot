#include <lvgl.h>
#include "app_lcd.h"
#include "app_ui.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "esp_camera.h"
#include "benchmark/lv_demo_benchmark.h"

static const char *TAG = "lcd";

/*********************
 *      DEFINES
 *********************/
#define EMOJI_NUMBER 7
#define LV_TICK_PERIOD_MS 1

SCREEN_CLASS screen;
static LGFX_Emma *_lgfxEmma = nullptr;
static SemaphoreHandle_t xGuiSemaphore;
static TaskHandle_t g_lvgl_task_handle;

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static lv_obj_t *camera_obj;
static bool gReturnFB = true;
static bool lvgl_ready = false;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);

LV_IMG_DECLARE(normal_gif);
LV_IMG_DECLARE(normal2_gif);
LV_IMG_DECLARE(listen_gif);
LV_IMG_DECLARE(listen2normal_gif);
LV_IMG_DECLARE(wakeup_gif);
LV_IMG_DECLARE(happy_gif);
LV_IMG_DECLARE(hurt_gif);

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
};

typedef struct
{
    const void *gif;
    uint16_t time;
    uint8_t repeat;
} emoji_list;

emoji_list em_list[EMOJI_NUMBER] =
{
    {&normal_gif, 5000, 1},
    {&normal2_gif, 5000, 1},
    {&listen_gif, 5000, 1},
    {&listen2normal_gif, 5000, 1},
    {&wakeup_gif, 5000, 1},
    {&happy_gif, 5000, 1},
    {&hurt_gif, 5000, 1},
};

static void lv_set_cam_area(void)
{
    camera_obj = lv_img_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);
    /*Set a background*/
    lv_obj_add_style(camera_obj, &style, 0);

    lv_obj_center(camera_obj);
    lv_obj_set_size(camera_obj, 240, 135);
}

void lv_camera_create(void)
{
    lv_set_cam_area();
}

static void lcd_task(void *arg)
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
            if (lvgl_ready)
            {
                // get camera farm
                img_dsc.data = frame->buf;
                lv_img_set_src(camera_obj, &img_dsc);
            }

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

esp_err_t AppLCD_Init(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
    screen.init();
    screen.setRotation(1);// 1 3 5 7
    screen.fillScreen(TFT_BLACK);
    screen.setBrightness(70);

    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;

    return ESP_OK;
}

IRAM_ATTR void disp_driver_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    _lgfxEmma->startWrite();
    _lgfxEmma->setAddrWindow(area->x1, area->y1, w, h);
    _lgfxEmma->pushColors((uint16_t *)&color_p->full, w * h, false);
    _lgfxEmma->endWrite();

    lv_disp_flush_ready(drv);
}

static lv_obj_t *gif_anim;
static uint8_t cnt = 0;

void next_frame_task_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    bool active = (bool) event->param;
    static bool is_wakeup = false;

    switch (code)
    {
    case LV_EVENT_READY:
    {
        printf("----gif play finsh----\n");

        if (!is_wakeup)
        {
            static uint8_t normal_cnt = 0;
            lv_gif_set_src(gif_anim, em_list[normal_cnt].gif);
            normal_cnt >= 2 ? normal_cnt = 0 : normal_cnt += 1;
            ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
        }

        break;
    }
    case LV_EVENT_VALUE_CHANGED:
    {
        ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
        active == true ? is_wakeup = true : is_wakeup = false;

        break;
    }
    default:
        break;
    }
}

void lv_emoji_create(void)
{
    gif_anim = lv_gif_create(lv_scr_act());
    lv_obj_add_event_cb(gif_anim, next_frame_task_cb, LV_EVENT_ALL, NULL);

    lv_gif_set_src(gif_anim, em_list[NORMAL_EMOJI].gif);
    lv_obj_align(gif_anim, LV_ALIGN_CENTER, 0, 0);

    ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
}

static void guiTask(void *pvParameter)
{
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    static lv_disp_draw_buf_t disp_buf;

    _lgfxEmma = &screen;

    lv_init();

    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1 != NULL);

    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    lv_disp_drv_register(&disp_drv);

    lv_emoji_create();

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args =
    {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    static TickType_t tick;
    tick = xTaskGetTickCount();
    lvgl_ready = true;

    while (1)
    {
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(30));
        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }
}

static void lv_tick_task(void *arg)
{
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

esp_err_t AppLVGL_run(void)
{
    esp_err_t ret = ESP_OK;
    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 4 * 1024, NULL, 2, &g_lvgl_task_handle, 0);
    if (result != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to create lvgl task");
        ret = ESP_FAIL;
    }
    return ret;
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

void ui_acquire(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    }
}

void ui_release(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreGive(xGuiSemaphore);
    }
}
