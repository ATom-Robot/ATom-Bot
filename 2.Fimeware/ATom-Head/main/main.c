#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"

#include <lvgl.h>
#include "lv_examples/lv_examples.h"

#include "st7789.h"
#include "camera_sensor.h"

#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_VER_RES_MAX / 2)
#define LV_TICK_PERIOD_MS 1

static TFT_t lcd_dev;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);

static IRAM_ATTR void disp_driver_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lcd_fill_array(&lcd_dev, area, color_map);

    lv_disp_flush_ready(drv);
}

static void guiTask(void *pvParameter)
{
    static lv_disp_buf_t disp_buf;

    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    lv_disp_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args =
    {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    lv_camera_create();
    // lv_demo_benchmark();

    static TickType_t tick;
    tick = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(CONFIG_LV_DISP_DEF_REFR_PERIOD));

        lv_task_handler();
    }
}

void app_main(void)
{
    printf("Hello-Atom-Bot!!\n");

    spi_master_init(&lcd_dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&lcd_dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 4096, NULL, 0, NULL, 1);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

static void lv_tick_task(void *arg)
{
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
