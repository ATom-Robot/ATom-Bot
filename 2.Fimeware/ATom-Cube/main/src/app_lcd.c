#include "app_lcd.h"

#include "freertos/task.h"
#include <lvgl.h>

#include "esp_err.h"
#include "benchmark/lv_demo_benchmark.h"

static TFT_t lcd_dev;

void AppLCD_Init(void)
{
    spi_master_init(&lcd_dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&lcd_dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    return ESP_OK;
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

    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    assert(buf1 != NULL);
    // lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    // assert(buf2 != NULL);

    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    lv_disp_drv_register(&disp_drv);

    lv_demo_benchmark();

    while (1)
    {
        delayMS(30);
        lv_timer_handler();
    }
}

void AppLVGL_run(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 4 * 1024, NULL, 5, NULL, 1);
    assert("Failed to create task" && result == (BaseType_t) 1);
}
