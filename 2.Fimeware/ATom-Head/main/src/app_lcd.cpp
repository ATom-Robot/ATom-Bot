#include "app_lcd.h"
#include "app_ui.h"
#include "esp_log.h"
#include "ui.h"

static const char *TAG = "lcd";

/*********************
 *      DEFINES
 *********************/
#define LV_TICK_PERIOD_MS 1

SCREEN_CLASS screen;
static LGFX_Emma *_lgfxEmma = nullptr;
static SemaphoreHandle_t xGuiSemaphore;
static TaskHandle_t g_lvgl_task_handle;

QueueHandle_t xQueueFrameI = NULL;
QueueHandle_t xQueueFrameO = NULL;

lv_obj_t *camera_obj = NULL;
bool gReturnFB = true;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);

esp_err_t AppLCD_Init(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
    screen.init();
    screen.setRotation(1);// 1 3 5 7
    screen.fillScreen(TFT_BLACK);
    screen.setBrightness(60);

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

    /* UI App Init*/
    ui_init();

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
