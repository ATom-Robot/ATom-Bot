#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <dirent.h>

#include "app_lcd.h"
#include "app_wifi.h"
#include "app_camera.h"
#include "app_speech.h"
#include "app_speaker.h"
#include "app_face_detection.hpp"

#include "app_player.h"
#include "app_shell.h"
#include "app_rtsp.h"
#include "stream_server.h"

static const char *TAG = "main";

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

    SPIFFS_Directory("/spiffs/");
}

extern "C" void app_main()
{
    QueueHandle_t xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    SPI_FS_Init();
    AppCamera_Init(PIXFORMAT_GRAYSCALE, FRAMESIZE_HQVGA, 2, xQueueLCDFrame);
    AppLCD_Init(xQueueLCDFrame, NULL, true);
    AppSpeech_Init();
    speaker_init();

    AppCamera_run();
    AppLVGL_run();
    AppSpeech_run();
    ESP_ERROR_CHECK(app_player_start("/spiffs/mp3"));
    app_wifi_main();

    // rtsp_server();
    assert(start_stream_server(xQueueLCDFrame, true) == ESP_OK);

    APP_Shell_loop();
}
