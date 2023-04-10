#include "esp_err.h"
#include "esp_log.h"

#include "app_lcd.h"
#include "app_wifi.h"
#include "app_camera.h"
#include "app_speech.h"
#include "app_speaker.h"
// #include "app_face_detection.hpp"
#include "app_apriltag.h"
#include "bsp_storage.h"

#include "app_player.h"
#include "app_shell.h"
#include "app_rtsp.h"
#include "stream_server.h"

static const char *TAG = "main";

extern "C" void app_main()
{
    QueueHandle_t xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    ESP_ERROR_CHECK(bsp_spiffs_init("model", "/srmodel", 4));
    ESP_ERROR_CHECK(bsp_spiffs_init("storage", "/spiffs", 2));

    AppCamera_Init(PIXFORMAT_GRAYSCALE, FRAMESIZE_240X240, 2, xQueueLCDFrame);
    AppLCD_Init(xQueueLCDFrame, NULL, true);
    AppSpeech_Init();
    speaker_init();

    AppCamera_run();
    AppLVGL_run();
    ESP_ERROR_CHECK(AppSpeech_run());
    ESP_ERROR_CHECK(app_player_start("/spiffs/mp3"));
    ESP_ERROR_CHECK(app_wifi_main());
    ESP_ERROR_CHECK(start_stream_server(xQueueLCDFrame, true));
    // // rtsp_server();
    APP_Shell_loop();
}
