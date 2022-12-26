#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "avifile.h"
#include "vidoplayer.h"

#include <../../lvgl/lvgl.h>

//#define CONFIG_AVI_AUDIO

#ifdef CONFIG_AVI_AUDIO
    #include "pwm_audio.h"
#endif

static const char *TAG = "avi player";

#define PLAYER_CHECK(a, str, ret)  if(!(a)) {                                             \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);      \
        return (ret);                                                                   \
        }

/**
 * TODO: how to recognize each stream id
 */
#define  T_vids  _REV(0x30306463)
#define  T_auds  _REV(0x30317762)

extern AVI_TypeDef AVI_file;

static uint32_t _REV(uint32_t value)
{
    return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |
           (value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}

#ifdef CONFIG_AVI_AUDIO
static void audio_init(void)
{
    pwm_audio_config_t pac;
    pac.duty_resolution    = LEDC_TIMER_10_BIT;
    pac.gpio_num_left      = 25;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
    pac.gpio_num_right     = -1;
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
    pac.ringbuf_len        = 1024 * 8;
    pwm_audio_init(&pac);

    pwm_audio_set_volume(0);
}
#endif

static uint32_t read_frame(FILE *file, uint8_t *buffer, uint32_t length, uint32_t *fourcc)
{
    AVI_CHUNK_HEAD head;
    fread(&head, sizeof(AVI_CHUNK_HEAD), 1, file);
    if (head.FourCC)
    {
        /* code */
    }
    *fourcc = head.FourCC;
    if (head.size % 2)
    {
        head.size++;    //奇数加1
    }
    if (length < head.size)
    {
        ESP_LOGE(TAG, "frame size too large");
        return 0;
    }

    fread(buffer, head.size, 1, file);
    return head.size;
}

static lv_img_dsc_t img_dsc =
{
    .header.always_zero = 0,
    .header.w = 240,
    .header.h = 135,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data_size = 240 * 135 * 2,
    .data = NULL,
};

void avi_play(const char *filename, void *param)
{
    FILE *avi_file;
    int ret;
    size_t  BytesRD;
    uint32_t  Strsize;
    uint32_t  Strtype;
    uint8_t *pbuffer;
    uint32_t buffer_size = 22 * 1024;

    avi_file = fopen(filename, "rb");
    if (avi_file == NULL)
    {
        ESP_LOGE(TAG, "Cannot open %s", filename);
        return;
    }

    pbuffer = malloc(buffer_size);
    if (pbuffer == NULL)
    {
        ESP_LOGE(TAG, "Cannot alloc memory for palyer");
        fclose(avi_file);
        return;
    }

    BytesRD = fread(pbuffer, 20480, 1, avi_file);
    ret = AVI_Parser(pbuffer, BytesRD);
    if (0 > ret)
    {
        ESP_LOGE(TAG, "parse failed (%d)", ret);
        return;
    }

#ifdef CONFIG_AVI_AUDIO
    audio_init();
    pwm_audio_set_param(AVI_file.auds_sample_rate, AVI_file.auds_bits, AVI_file.auds_channels);
    pwm_audio_start();
#endif

    uint16_t img_width = AVI_file.vids_width;
    uint16_t img_height = AVI_file.vids_height;
    ESP_LOGI(TAG, "img_width:%d, img_height:%d", img_width, img_height);
    uint8_t *img_rgb888 = heap_caps_malloc(img_width * img_height * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (NULL == img_rgb888)
    {
        ESP_LOGE(TAG, "malloc for rgb888 failed");
        goto EXIT;
    }

    fseek(avi_file, AVI_file.movi_start, SEEK_SET); // 偏移到 movi list
    Strsize = read_frame(avi_file, pbuffer, buffer_size, &Strtype);
    BytesRD = Strsize + 8;

    static int64_t last_frame = 0;
    last_frame = esp_timer_get_time();

    while (1)   //播放循环
    {
        if (BytesRD >= AVI_file.movi_size)
        {
            ESP_LOGI(TAG, "paly end");
            break;
        }
        if (Strtype == T_vids)
        {
            mjpegdraw(pbuffer, Strsize, img_rgb888);

            img_dsc.data = (uint8_t *)img_rgb888;
            lv_img_set_src(param, &img_dsc);

            int64_t fr_end = esp_timer_get_time();
            int64_t frame_time = fr_end - last_frame;
            last_frame = fr_end;
            frame_time /= 1000;
            ESP_LOGI(TAG, "(%.1ffps)", 1000.0 / (uint32_t)frame_time);
        }
        else if (Strtype == T_auds)   //音频输出
        {
#ifdef CONFIG_AVI_AUDIO
            size_t cnt;
            pwm_audio_write((uint8_t *)pbuffer, Strsize, &cnt, 500 / portTICK_PERIOD_MS);
#endif
        }
        else
        {
            ESP_LOGE(TAG, "unknow frame");
            break;
        }
        Strsize = read_frame(avi_file, pbuffer, buffer_size, &Strtype); //读入整帧
        ESP_LOGD(TAG, "type=%x, size=%d", Strtype, Strsize);
        BytesRD += Strsize + 8;
    }
EXIT:
#ifdef CONFIG_AVI_AUDIO
    pwm_audio_deinit();
#endif
    free(img_rgb888);
    free(pbuffer);
    fclose(avi_file);
}

static void aviTask(void *pvParameter)
{
    avi_play("/spiffs/5.avi", pvParameter);

    // never do this
    while (1)
    {
        vTaskDelay(2000);
    }
}

void lv_set_avi_area(lv_obj_t *obj)
{
    obj = lv_img_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);

    /*Set a background*/
    lv_style_set_img_recolor(&style, lv_color_black());
    lv_style_set_img_recolor_opa(&style, LV_OPA_COVER);
    lv_style_set_img_opa(&style, LV_OPA_COVER);
    lv_obj_add_style(obj, &style, 0);

    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 240, 135);

    BaseType_t result = xTaskCreatePinnedToCore(aviTask, "avi", 1024 * 5, (lv_obj_t *)obj, 2, NULL, 1);
    assert("Failed to create task" && result == (BaseType_t) 1);
}

void lv_avi_create(void)
{
    static lv_obj_t *avi_obj;
    lv_set_avi_area(avi_obj);
}

