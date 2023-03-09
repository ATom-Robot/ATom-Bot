#include "app_lcd.h"

#include "freertos/task.h"
#include <lvgl.h>

#include "esp_camera.h"
#include "esp_err.h"

static TFT_t lcd_dev;

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static bool gReturnFB = true;
static bool lvgl_ready = false;

//
static lv_obj_t *camera_obj;

static lv_img_dsc_t img_dsc =
{
    .header.always_zero = 0,
    .header.w = 240,
    .header.h = 135,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data_size = 240 * 135 * 2,
    .data = NULL,
};

static void lv_set_cam_area(void)
{
    camera_obj = lv_img_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);

    /*Set a background*/
    lv_style_set_img_recolor(&style, lv_color_black());
    lv_style_set_img_recolor_opa(&style, LV_OPA_0);
    lv_style_set_img_opa(&style, 255);
    lv_obj_add_style(camera_obj, &style, 0);

    lv_obj_set_pos(camera_obj, 0, 0);
    lv_obj_set_size(camera_obj, 240, 135);
}

void lv_camera_create(void)
{
    lv_set_cam_area();
}
//

static void lcd_task(void *arg)
{
    camera_fb_t *frame = NULL;

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
    spi_master_init(&lcd_dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&lcd_dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;

    return ESP_OK;
}

void AppLCD_run(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(lcd_task, "lcd", 4 * 1024, NULL, 2, NULL, 0);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

IRAM_ATTR void disp_driver_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lcd_fill_array(&lcd_dev, area, color_map);

    lv_disp_flush_ready(drv);
}

static void guiTask(void *pvParameter)
{
    static lv_disp_draw_buf_t disp_buf;

    lv_init();

    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1 != NULL);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2 != NULL);

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    lv_disp_drv_register(&disp_drv);

    lv_camera_create();
    // lv_avi_create();
    // lv_demo_benchmark();

    static TickType_t tick;
    tick = xTaskGetTickCount();

    lvgl_ready = true;

    while (1)
    {
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(30));
        lv_timer_handler();
    }
}

void AppLVGL_run(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 3 * 1024, NULL, 5, NULL, 1);
    assert("Failed to create task" && result == (BaseType_t) 1);
}
