#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"

#include <lvgl.h>
// #include "lv_examples/lv_examples.h"

#include "st7789.h"
#include "camera_sensor.h"

#include "esp_spiffs.h"
#include "esp_vfs.h"

#include "vidoplayer.h"

static const char *TAG = "main";

#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_VER_RES_MAX / 2)
#define LV_TICK_PERIOD_MS 1

// static lv_color_t buf1[DISP_BUF_SIZE];
// static lv_color_t buf2[DISP_BUF_SIZE];
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);

static TFT_t lcd_dev;

extern SemaphoreHandle_t BinarySemaphore;
static IRAM_ATTR void disp_driver_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    xSemaphoreGive(BinarySemaphore);

    lcd_fill_array(&lcd_dev, area, color_map);
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

    // lv_camera_create();
    lv_avi_create();
    // lv_demo_benchmark();

    static TickType_t tick;
    tick = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(CONFIG_LV_DISP_DEF_REFR_PERIOD));

        lv_task_handler();
    }
}

static void SPIFFS_Directory(char *path)
{
    DIR *dir = opendir(path);
    assert(dir != NULL);
    while (true)
    {
        struct dirent *pe = readdir(dir);
        if (!pe) break;
        ESP_LOGI(__FUNCTION__, "d_name=%s d_ino=%d d_type=%x", pe->d_name, pe->d_ino, pe->d_type);
    }
    closedir(dir);
}

static void SPI_FS_Init(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

void app_main(void)
{
    printf("Hello-Atom-Bot!!\n");

    SPI_FS_Init();
    SPIFFS_Directory("/spiffs/");

    spi_master_init(&lcd_dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&lcd_dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    BaseType_t result = xTaskCreatePinnedToCore(guiTask, "gui", 4096, NULL, 5, NULL, 1);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

static void lv_tick_task(void *arg)
{
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
