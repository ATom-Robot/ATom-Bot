#include "app_lcd.h"
#include <lvgl.h>
#include "esp_camera.h"
#include "benchmark/lv_demo_benchmark.h"
#include "rlottie/lv_rlottie.h"

SCREEN_CLASS screen;
static LGFX_Emma *_lgfxEmma = nullptr;

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static lv_obj_t *camera_obj;
static bool gReturnFB = true;
static bool lvgl_ready = false;

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

void AppLCD_Init(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
    screen.init();
    screen.setRotation(1);// 1 3 5 7
    screen.fillScreen(TFT_BLACK);
    screen.setBrightness(70);

    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;
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

void lv_example_gif_1(void)
{
    LV_IMG_DECLARE(gif_normal);

    lv_obj_t * img;
    img = lv_gif_create(lv_scr_act());
    lv_gif_set_src(img, &gif_normal);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}

static void guiTask(void *pvParameter)
{
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

    // lv_avi_create();
    // lv_camera_create();
    // lv_demo_benchmark();
    lv_example_gif_1();

    static TickType_t tick;
    tick = xTaskGetTickCount();
    lvgl_ready = true;

    while (1)
    {
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(30));
        lv_task_handler();
    }
}

// void AppLCD_run(void)
// {
//     BaseType_t result = xTaskCreatePinnedToCore(lcd_task, "lcd", 2 * 1024, NULL, 2, NULL, 0);
//     assert("Failed to create task" && result == (BaseType_t) 1);
// }

void AppLVGL_run(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 4 * 1024, NULL, 5, NULL, 0);
    assert("Failed to create task" && result == (BaseType_t) 1);
}
