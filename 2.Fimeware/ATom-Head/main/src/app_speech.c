#include "driver/i2s.h"
#include "esp_log.h"

#include "dl_lib_coefgetter_if.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_afe_sr_iface.h"
#include "model_path.h"

#include "app_speech.h"

static const char *TAG = "App/Speech";

srmodel_list_t *models = NULL;
static esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t *afe_data = NULL;
static command_word_t command;

static int detect_flag = 0;
static volatile bool task_flag = false;

static esp_err_t bsp_i2s_init(i2s_port_t i2s_num)
{
    esp_err_t ret_val = ESP_OK;

    i2s_config_t i2s_config = I2S_CONFIG_DEFAULT();

    i2s_pin_config_t pin_config =
    {
        .bck_io_num = GPIO_I2S_SCLK,
        .ws_io_num = GPIO_I2S_LRCK,
        .data_out_num = GPIO_I2S_DOUT,
        .data_in_num = GPIO_I2S_SDIN,
        .mck_io_num = GPIO_I2S_MCLK,
    };

    ret_val |= i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    ret_val |= i2s_set_pin(i2s_num, &pin_config);
    ret_val |= i2s_zero_dma_buffer(i2s_num);

    return ret_val;
}

static esp_err_t bsp_i2s_deinit(i2s_port_t i2s_num)
{
    esp_err_t ret_val = ESP_OK;

    ret_val |= i2s_stop(I2S_NUM_0);
    ret_val |= i2s_driver_uninstall(i2s_num);

    return ret_val;
}

esp_err_t bsp_get_feed_data(int16_t *buffer, int buffer_len)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_read;
    int audio_chunksize = buffer_len / (sizeof(int32_t));
    ret = i2s_read(I2S_NUM_1, buffer, buffer_len, &bytes_read, portMAX_DELAY);

    int32_t *tmp_buff = buffer;
    for (int i = 0; i < audio_chunksize; i++)
    {
        tmp_buff[i] = tmp_buff[i] >> 14; // 32:8为有效位， 8:0为低8位， 全为0， AFE的输入为16位语音数据，拿29：13位是为了对语音信号放大。
    }

    return ret;
}

int bsp_get_feed_channel(void)
{
    return ADC_I2S_CHANNEL;
}

static void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = bsp_get_feed_channel();
    int16_t *i2s_buff = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(i2s_buff);
    size_t bytes_read;

    while (task_flag)
    {
        bsp_get_feed_data(i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff)
    {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

static void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int16_t *audio_buffer = (int16_t *)heap_caps_malloc(afe_chunksize * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (NULL == audio_buffer)
    {
        esp_system_abort("No mem for audio buffer");
    }

    static const esp_mn_iface_t *multinet = &MULTINET_MODEL;
    model_iface_data_t *model_data = multinet->create((const model_coeff_getter_t *)&MULTINET_COEFF, 5760);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    int chunk_num = multinet->get_samp_chunknum(model_data);
    assert(mu_chunksize == afe_chunksize);

    printf("------------detect start------------\n");

    while (task_flag)
    {
        int res =  afe_handle->fetch(afe_data, audio_buffer);
        if (res == AFE_FETCH_WWE_DETECTED)
        {
            ESP_LOGI(TAG, ">>> Say your command <<<");
            detect_flag = 1;
            afe_handle->disable_wakenet(afe_data);
        }

        if (detect_flag)
        {
            command = multinet->detect(model_data, audio_buffer);

            if (command == COMMAND_NOT_DETECTED)
                continue;
            else if (command == COMMAND_TIMEOUT)
            {
                afe_handle->enable_wakenet(afe_data);
                // self->afe_handle->enable_aec(afe_data);

                detect_flag = 0;
                ESP_LOGI(TAG, ">>> Waiting to be waken up <<<");
            }
            else
            {
                ESP_LOGI(TAG, "Command: %d", command);

                afe_handle->enable_wakenet(afe_data);
                // self->afe_handle->enable_aec(afe_data);
                detect_flag = 0;
                ESP_LOGI(TAG, ">>> Waiting to be waken up <<<");
                command = COMMAND_TIMEOUT;
            }
        }
    }
    if (model_data)
    {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    vTaskDelete(NULL);
}

void AppSpeech_Init(void)
{
    esp_err_t res = -1;
    res = bsp_i2s_init(I2S_NUM_1);
    assert(res != ESP_FAIL);

    models = esp_srmodel_init("model");
    if (models != NULL)
    {
        for (int i = 0; i < models->num; i++)
        {
            printf("Load: %s\n", models->model_name[i]);
        }
    }

    afe_handle = &esp_afe_sr_1mic;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.aec_init = false;
    afe_config.se_init = false;
    afe_config.vad_init = false;
    afe_config.afe_ringbuf_size = 10;
    afe_config.alloc_from_psram = AFE_PSRAM_HIGH_COST;

    afe_data = afe_handle->create_from_config(&afe_config);
}

void AppSpeech_run(void)
{
    task_flag = true;

    BaseType_t result1 = xTaskCreatePinnedToCore((TaskFunction_t)feed_Task, "App/SR/Feed", 3 * 1024, afe_data, 2, NULL, 0);
    assert("Failed to create task" && result1 == (BaseType_t) 1);

    BaseType_t result2 = xTaskCreatePinnedToCore((TaskFunction_t)detect_Task, "App/SR/Detect", 5 * 1024, afe_data, 2, NULL, 0);
    assert("Failed to create task" && result2 == (BaseType_t) 1);
}