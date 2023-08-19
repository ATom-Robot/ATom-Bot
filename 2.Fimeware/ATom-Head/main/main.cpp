#include "esp_err.h"
#include "esp_log.h"
#include "bsp_storage.h"

#include "app_lcd.h"
#include "app_wifi.h"
#include "app_camera.h"
#include "app_speech.h"
#include "app_speaker.h"
// #include "app_face_detection.hpp"
#include "app_apriltag.h"
#include "app_player.h"
#include "app_shell.h"
#include "app_rtsp.h"
#include "app_joint.h"
#include "app_uart.h"
#include "stream_server.h"

extern "C" void app_main()
{
    QueueHandle_t xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    ESP_ERROR_CHECK(bsp_spiffs_init("model", "/srmodel", 4));
    ESP_ERROR_CHECK(bsp_spiffs_init("storage", "/spiffs", 4));

    ESP_ERROR_CHECK(AppCamera_Init(PIXFORMAT_JPEG, FRAMESIZE_QVGA, 2, xQueueLCDFrame));
    ESP_ERROR_CHECK(AppLCD_Init(xQueueLCDFrame, NULL, true));
    ESP_ERROR_CHECK(AppSpeaker_Init());
    ESP_ERROR_CHECK(AppSpeech_Init());
    ESP_ERROR_CHECK(APPUart_Init());
    ESP_ERROR_CHECK(joint_i2c_init());

    ESP_ERROR_CHECK(app_player_start("/spiffs/mp3"));
    ESP_ERROR_CHECK(AppSpeech_run());
    ESP_ERROR_CHECK(AppLVGL_run());
    // ESP_ERROR_CHECK(AppCamera_run());
    // ESP_ERROR_CHECK(app_wifi_main());
    // ESP_ERROR_CHECK(start_stream_server(xQueueLCDFrame, true));
    // // rtsp_server();
    ESP_ERROR_CHECK(AppShell_run());
}
