#include "OV2640.h"
#include "platglue.h"

#define TAG "OV2640"

camera_config_t esp32cam_config
{
    .pin_pwdn = -1,
    .pin_reset = -1,

    .pin_xclk = 15,

    .pin_sscb_sda = 4,
    .pin_sscb_scl = 5,

    .pin_d7 = 16,
    .pin_d6 = 17,
    .pin_d5 = 18,
    .pin_d4 = 12,
    .pin_d3 = 10,
    .pin_d2 = 8,
    .pin_d1 = 9,
    .pin_d0 = 11,
    .pin_vsync = 6,
    .pin_href = 7,
    .pin_pclk = 13,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_HQVGA,
    .jpeg_quality = 10, //0-63 lower numbers are higher quality
    .fb_count = 2,      // if more than one i2s runs in continous mode.  Use only with jpeg
#ifndef ARDUINO_ARCH_ESP32
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1
#endif
};

void OV2640::done(void)
{
    DEBUG_PRINT("OV240: done\n");
    if (fb)
    {
        DEBUG_PRINT("OV240: done fb return\n");
        //return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);
        fb = NULL;
    }
}

void OV2640::run(void)
{
    DEBUG_PRINT("OV240: run\n");
    if (fb)
    {
        DEBUG_PRINT("OV240: run fb return\n");
        //return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);
    }

    DEBUG_PRINT("OV240: run fb get\n");
    fb = esp_camera_fb_get();
    DEBUG_PRINT("OV240: run fb gotten\n");
}

void OV2640::runIfNeeded(void)
{
    if (!fb)
    {
        DEBUG_PRINT("OV240: runIfNeeded running\n");
        run();
    }
}

int OV2640::getWidth(void)
{
    runIfNeeded();
    return fb->width;
}

int OV2640::getHeight(void)
{
    runIfNeeded();
    return fb->height;
}

size_t OV2640::getSize(void)
{
    runIfNeeded();
    if (!fb)
    {
        return 0; // FIXME - this shouldn't be possible but apparently the new cam board returns null sometimes?
    }
    return fb->len;
}

uint8_t *OV2640::getfb(void)
{
    runIfNeeded();
    if (!fb)
    {
        return NULL; // FIXME - this shouldn't be possible but apparently the new cam board returns null sometimes?
    }

    return fb->buf;
}

framesize_t OV2640::getFrameSize(void)
{
    return _cam_config.frame_size;
}

void OV2640::setFrameSize(framesize_t size)
{
    _cam_config.frame_size = size;
}

pixformat_t OV2640::getPixelFormat(void)
{
    return _cam_config.pixel_format;
}

void OV2640::setPixelFormat(pixformat_t format)
{
    switch (format)
    {
    case PIXFORMAT_RGB565:
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
    case PIXFORMAT_JPEG:
        _cam_config.pixel_format = format;
        break;
    default:
        _cam_config.pixel_format = PIXFORMAT_GRAYSCALE;
        break;
    }
}

esp_err_t OV2640::init(camera_config_t config)
{
    memset(&_cam_config, 0, sizeof(_cam_config));
    memcpy(&_cam_config, &config, sizeof(config));

    //do a long powerdown of the camera, the esp32-camera code power downs for 10ms, which may not be enough
    if (_cam_config.pin_pwdn >= 0)
    {
        gpio_config_t conf;
        memset(&conf, 0, sizeof(gpio_config_t));
        conf.pin_bit_mask = 1LL << _cam_config.pin_pwdn;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // this will take a full second, should not cause any watchdog issues
        gpio_set_level((gpio_num_t)_cam_config.pin_pwdn, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)_cam_config.pin_pwdn, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    esp_err_t err = esp_camera_init(&_cam_config);
    if (err != ESP_OK)
    {
        DEBUG_PRINT("Camera probe failed with error 0x%x", err);
        return err;
    }

    return ESP_OK;
}
