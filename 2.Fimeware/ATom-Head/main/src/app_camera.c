#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "app_camera.h"

static const char *TAG = "app/camera";

static QueueHandle_t queue_output = NULL;

static void camera_task(void *param)
{
    ESP_LOGI(TAG, "Start");
    while (1)
    {
        if (queue_output == NULL)
            break;

        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
            xQueueSend(queue_output, &frame, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "Stop");
    vTaskDelete(NULL);
}

void AppCamera_Init(const pixformat_t pixel_fromat,
                    const framesize_t frame_size,
                    const uint8_t fb_count,
                    const QueueHandle_t queue_o)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_sharpness(s, 2);
    s->set_awb_gain(s, 2);

    queue_output = queue_o;
}

void AppCamera_run(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(camera_task, "cam", 4 * 1024, NULL, 2, NULL, 0);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

