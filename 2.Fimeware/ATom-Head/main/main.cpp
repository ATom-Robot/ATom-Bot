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
#include "app_tcpserver.h"
#include "stream_server.h"

extern "C" void app_main()
{
    QueueHandle_t xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    ESP_ERROR_CHECK(bsp_spiffs_init("model", "/srmodel", 4));
    ESP_ERROR_CHECK(bsp_spiffs_init("storage", "/spiffs", 4));

    ESP_ERROR_CHECK(App_Camera_Init(PIXFORMAT_JPEG, FRAMESIZE_QVGA, 3, xQueueLCDFrame));
    ESP_ERROR_CHECK(App_Lcd_Init(xQueueLCDFrame, NULL, true));
    ESP_ERROR_CHECK(App_Speech_Init());
    ESP_ERROR_CHECK(App_Speaker_Init());
    ESP_ERROR_CHECK(APP_Uart_Init());
    ESP_ERROR_CHECK(App_Joint_Init());

    ESP_ERROR_CHECK(App_Speech_run());
    ESP_ERROR_CHECK(App_Camera_run());
    ESP_ERROR_CHECK(App_Lvgl_run());
    ESP_ERROR_CHECK(App_Wifi_run());
    ESP_ERROR_CHECK(APP_TcpServer_run());
    ESP_ERROR_CHECK(App_Player_run("/spiffs/mp3"));
    ESP_ERROR_CHECK(App_Stream_run(xQueueLCDFrame, true));
    // rtsp_server();
    ESP_ERROR_CHECK(App_Shell_run());
}
