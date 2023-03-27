#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "app_lcd.h"
#include "app_camera.h"
#include "app_speech.h"
#include "app_speaker.h"
#include "app_face_detection.hpp"

#include "app_shell.h"
#include "app_audio.h"
#include "audio_player.h"

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

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    ESP_LOGI(TAG, "mute setting %d", setting);
    return ESP_OK;
}

extern "C" void app_main()
{
    SPI_FS_Init();

    // QueueHandle_t xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    QueueHandle_t xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    AppSpeech_Init();
    // AppCamera_Init(PIXFORMAT_RGB565, FRAMESIZE_HQVGA, 2, xQueueAIFrame);
    AppCamera_Init(PIXFORMAT_RGB565, FRAMESIZE_HQVGA, 2, xQueueLCDFrame);
    // register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
    AppLCD_Init(xQueueLCDFrame, NULL, true);

    AppCamera_run();
    AppLVGL_run();
    AppLCD_run();
    AppSpeech_run();

    // const char mp3_start[] = "/spiffs/404-41-4.mp3";

    // FILE *fp = fopen(mp3_start, "rb");
    // if (!fp)
    // {
    //     ESP_LOGE(TAG, "unable to open '%s'", mp3_start);
    //     return;
    // }

    // speaker_init();

    // audio_player_config_t config = { .mute_fn = audio_mute_function,
    //                                  .clk_set_fn = audio_i2s_reconfig_clk,
    //                                  .write_fn = audio_i2s_write,
    //                                  .priority = 5
    //                                };
    // esp_err_t ret = audio_player_new(config);

    // audio_player_play(fp);

    APP_Shell_loop();
}
