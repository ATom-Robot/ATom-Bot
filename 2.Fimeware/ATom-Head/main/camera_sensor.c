
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_camera.h"

#define USE_WEB_CAMERA      0

#if USE_WEB_CAMERA == 1
    #include "esp_http_server.h"
    #include "esp_timer.h"
    #include "app_wifi.h"
#else
    #include <lvgl.h>
#endif

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1

#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13
#define XCLK_GPIO_NUM     15

#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11

static const char *TAG = "test camera";

static camera_config_t camera_config =
{
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .fb_location = CAMERA_FB_IN_PSRAM,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_HQVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 2,      //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t Init_Camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();

    return ESP_OK;
}

#if USE_WEB_CAMERA == 1

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t *_jpg_buf;
    char *part_buf[64];
    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        if (fb->format != PIXFORMAT_JPEG)
        {
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if (!jpeg_converted)
            {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (fb->format != PIXFORMAT_JPEG)
        {
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK)
        {
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
                 (uint32_t)(_jpg_buf_len / 1024),
                 (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

httpd_uri_t stream =
{
    .uri = "/stream",
    .method = HTTP_GET,
    .handler   = jpg_stream_httpd_handler,
    .user_ctx  = "stream"
};

static void http_test_task(void *param)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &stream);
    }
    else
    {
        ESP_LOGI(TAG, "Error start server");
    }

    ESP_LOGI(TAG, "Http start Success!!");

    vTaskDelete(NULL);
}

void app_main(void)
{
    app_wifi_main();

    Init_Camera();

    app_wifi_wait_connected();

    xTaskCreate(http_test_task, "cam", 2048 * 4, NULL, 5, NULL);
}
#else
static lv_img_dsc_t img_dsc =
{
    .header.always_zero = 0,
    .header.w = 240,
    .header.h = 135,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data_size = 240 * 135 * 2,
    .data = NULL,
};

static void lvgl_camera_task(void *param)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    while (1)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        else
        {
            img_dsc.data = fb->buf;
            lv_img_set_src(param, &img_dsc);
            esp_camera_fb_return(fb);
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;

        // ESP_LOGI(TAG, "(%.1ffps)", 1000.0 / (uint32_t)frame_time);
    }
}

void lv_set_cam_area(lv_obj_t *obj)
{
    obj = lv_img_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);

    /*Set a background*/
    lv_style_set_img_recolor(&style, lv_color_black());
    lv_style_set_img_recolor_opa(&style, LV_OPA_0);
    lv_style_set_img_opa(&style, 255);
    lv_obj_add_style(obj, &style, 0);

    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 240, 135);

    BaseType_t result = xTaskCreatePinnedToCore(lvgl_camera_task, "cam", 4096, (lv_obj_t *)obj, 2, NULL, 0);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

void lv_camera_create(void)
{
    assert(Init_Camera() == ESP_OK);
    ESP_LOGI(TAG, "Camera Init Success");

    static lv_obj_t *camera_obj;
    lv_set_cam_area(camera_obj);
}
#endif
