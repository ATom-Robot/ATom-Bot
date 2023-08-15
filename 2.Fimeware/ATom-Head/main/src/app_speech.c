#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <sys/time.h>

#include "dl_lib_coefgetter_if.h"
#include "esp_task_wdt.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_afe_sr_iface.h"
#include "model_path.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "model_path.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "app_speech.h"
#include "app_sr_handler.h"

#define NEED_DELETE BIT0
#define FEED_DELETED BIT1
#define DETECT_DELETED BIT2

static const char *TAG = "App/Speech";
typedef struct
{
    sr_language_t lang;
    model_iface_data_t *model_data;
    const esp_mn_iface_t *multinet;
    const esp_afe_sr_iface_t *afe_handle;
    esp_afe_sr_data_t *afe_data;
    int16_t *afe_in_buffer;
    int16_t *afe_out_buffer;
    SLIST_HEAD(sr_cmd_list_t, sr_cmd_t)
    cmd_list;
    uint8_t cmd_num;
    TaskHandle_t feed_task;
    TaskHandle_t detect_task;
    TaskHandle_t handle_task;
    QueueHandle_t result_que;
    EventGroupHandle_t event_group;

    FILE *fp;
    bool b_record_en;
} sr_data_t;

static sr_data_t *g_sr_data = NULL;

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

    ret_val |= i2s_stop(i2s_num);
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

/**
 * @brief all default commands
 */
static const sr_cmd_t g_default_cmd_info[] =
{
    {SR_CMD_PLAY, SR_LANG_CN, 0, "播放音乐", "bo fang yin yue", {NULL}},
    {SR_CMD_NEXT, SR_LANG_CN, 0, "下一曲", "xia yi qv", {NULL}},
    {SR_CMD_PAUSE, SR_LANG_CN, 0, "暂停", "zan ting", {NULL}},
    {SR_CMD_PAUSE, SR_LANG_CN, 0, "暂停播放", "zan ting bo fang", {NULL}},
    {SR_CMD_PAUSE, SR_LANG_CN, 0, "停止播放", "ting zhi bo fang", {NULL}},
};

static void feed_Task(void *pvParam)
{
    const esp_afe_sr_iface_t *afe_handle = g_sr_data->afe_handle;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)pvParam;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = bsp_get_feed_channel();
    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);
    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (NULL == audio_buffer)
    {
        esp_system_abort("No mem for audio buffer");
    }
    g_sr_data->afe_in_buffer = audio_buffer;
    while (task_flag)
    {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group))
        {
            xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
            vTaskDelete(NULL);
        }

        bsp_get_feed_data(audio_buffer, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, audio_buffer);
    }
    if (audio_buffer)
    {
        heap_caps_free(audio_buffer);
        audio_buffer = NULL;
    }
    vTaskDelete(NULL);
}

static void detect_Task(void *pvParam)
{
    bool detect_flag = false;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)pvParam;

    /* Allocate buffer for detection */
    size_t afe_chunk_size = g_sr_data->afe_handle->get_fetch_chunksize(afe_data);
    g_sr_data->afe_out_buffer = heap_caps_malloc(afe_chunk_size * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (NULL == g_sr_data->afe_out_buffer)
    {
        ESP_LOGE(TAG, "Expect : %zu, avaliable : %zu",
                 afe_chunk_size * sizeof(int16_t),
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        esp_system_abort("No mem for detect buffer");
    }

    /* Check for chunk size */
    if (afe_chunk_size != g_sr_data->multinet->get_samp_chunksize(g_sr_data->model_data))
    {
        esp_system_abort("Invalid chunk size");
    }

    printf("------------detect start------------\n");

    while (task_flag)
    {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group))
        {
            xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
            vTaskDelete(g_sr_data->handle_task);
            vTaskDelete(NULL);
        }

        afe_fetch_mode_t ret_val = g_sr_data->afe_handle->fetch(afe_data, g_sr_data->afe_out_buffer);

        if (AFE_FETCH_WWE_DETECTED == ret_val)
        {
            ESP_LOGI(TAG, ">>> Say your command <<<");
            detect_flag = true;
            g_sr_data->afe_handle->disable_wakenet(afe_data);

            sr_result_t result =
            {
                .fetch_mode = ret_val,
                .state = ESP_MN_STATE_DETECTING,
                .command_id = 0,
            };
            xQueueSend(g_sr_data->result_que, &result, 0);
        }

        if (true == detect_flag)
        {
            esp_mn_state_t mn_state = ESP_MN_STATE_DETECTING;
            mn_state = g_sr_data->multinet->detect(g_sr_data->model_data, g_sr_data->afe_out_buffer);

            if (mn_state == ESP_MN_STATE_DETECTING)
                continue;
            if (ESP_MN_STATE_TIMEOUT == mn_state)
            {
                ESP_LOGW(TAG, "Time out");
                sr_result_t result =
                {
                    .fetch_mode = ret_val,
                    .state = mn_state,
                    .command_id = 0,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }
            else
            {
                int sr_command_id = mn_state;
                ESP_LOGI(TAG, "Deteted command : %d", sr_command_id);
                sr_result_t result =
                {
                    .fetch_mode = ret_val,
                    .state = mn_state,
                    .command_id = sr_command_id,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }
        }
    }
    if (g_sr_data->model_data)
    {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
        g_sr_data->model_data = NULL;
    }
    vTaskDelete(NULL);
}

esp_err_t AppSpeech_Init(void)
{
    esp_err_t res = ESP_OK;
    res = bsp_i2s_init(I2S_NUM_1);
    assert(res != ESP_FAIL);
    return res;
}

esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    xQueueReceive(g_sr_data->result_que, result, xTicksToWait);
    return ESP_OK;
}

const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, NULL, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, NULL, TAG, "cmd id out of range");

    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        if (id == it->id)
        {
            return it;
        }
    }
    ESP_RETURN_ON_FALSE(NULL != it, NULL, TAG, "can't find cmd id:%d", id);
    return NULL;
}

esp_err_t app_sr_update_cmds(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    char *cmd_str = heap_caps_calloc(g_sr_data->cmd_num, SR_CMD_PHONEME_LEN_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != cmd_str, ESP_ERR_NO_MEM, TAG, "memory for sr cmd str is not enough");

    uint32_t count = 0;
    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        it->id = count++;
        strcat(cmd_str, it->phoneme);
        strcat(cmd_str, ";");
    }

    ESP_LOGI(TAG, "New %d command set to :[%s]", count, cmd_str);
    g_sr_data->multinet->reset(g_sr_data->model_data, cmd_str, -1);

    heap_caps_free(cmd_str);
    return ESP_OK;
}

esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG, "pointer of cmd is invaild");
    ESP_RETURN_ON_FALSE(200 >= g_sr_data->cmd_num, ESP_ERR_INVALID_STATE, TAG, "cmd is full");
    sr_cmd_t *item = (sr_cmd_t *)heap_caps_calloc(1, sizeof(sr_cmd_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != item, ESP_ERR_NO_MEM, TAG, "memory for sr cmd is not enough");
    memcpy(item, cmd, sizeof(sr_cmd_t));
    item->next.sle_next = NULL;
#if 1 // insert after
    sr_cmd_t *last = SLIST_FIRST(&g_sr_data->cmd_list);
    if (last == NULL)
    {
        SLIST_INSERT_HEAD(&g_sr_data->cmd_list, item, next);
    }
    else
    {
        sr_cmd_t *it;
        while ((it = SLIST_NEXT(last, next)) != NULL)
        {
            last = it;
        }
        SLIST_INSERT_AFTER(last, item, next);
    }
#else // insert head
    SLIST_INSERT_HEAD(&g_sr_data->cmd_list, it, next);
#endif
    g_sr_data->cmd_num++;
    return ESP_OK;
}

esp_err_t app_sr_set_language(sr_language_t new_lang)
{
    if (g_sr_data->model_data)
    {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
    }
    g_sr_data->multinet = &MULTINET_MODEL;
    g_sr_data->model_data = g_sr_data->multinet->create((const model_coeff_getter_t *)&MULTINET_COEFF, 5760);

    // remove all command
    sr_cmd_t *it;
    while (!SLIST_EMPTY(&g_sr_data->cmd_list))
    {
        it = SLIST_FIRST(&g_sr_data->cmd_list);
        SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
        heap_caps_free(it);
    }

    uint8_t cmd_number = 0;
    // count command number
    for (size_t i = 0; i < sizeof(g_default_cmd_info) / sizeof(sr_cmd_t); i++)
    {
        cmd_number++;
        app_sr_add_cmd(&g_default_cmd_info[i]);
    }
    ESP_LOGI(TAG, "cmd_number=%d", cmd_number);
    return app_sr_update_cmds(); /* Reset command list */
}

esp_err_t AppSpeech_run(void)
{
    esp_err_t ret = ESP_OK;
    task_flag = true;

    ESP_RETURN_ON_FALSE(NULL == g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR already running");

    g_sr_data = heap_caps_calloc(1, sizeof(sr_data_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_NO_MEM, TAG, "Failed create sr data");

    g_sr_data->result_que = xQueueCreate(3, sizeof(sr_result_t));
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG, "Failed create result queue");

    g_sr_data->event_group = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG, "Failed create event_group");

    SLIST_INIT(&g_sr_data->cmd_list);

    esp_task_wdt_reset();
    g_sr_data->afe_handle = &esp_afe_sr_1mic;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.aec_init = false;
    afe_config.se_init = false;
    afe_config.vad_init = false;
    afe_config.alloc_from_psram = AFE_PSRAM_HIGH_COST;
    g_sr_data->afe_data = g_sr_data->afe_handle->create_from_config(&afe_config);
    g_sr_data->lang = SR_LANG_CN;
    ret = app_sr_set_language(g_sr_data->lang);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ESP_FAIL, err, TAG, "Failed to set language");

    BaseType_t ret_val = xTaskCreatePinnedToCore((TaskFunction_t)feed_Task, "App/SR/Feed", 4 * 1024, g_sr_data->afe_data, 5, &g_sr_data->feed_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG, "Failed create audio feed task");

    ret_val = xTaskCreatePinnedToCore((TaskFunction_t)detect_Task, "App/SR/Detect", 6 * 1024, g_sr_data->afe_data, 5, &g_sr_data->detect_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG, "Failed create audio detect task");

    ret_val = xTaskCreatePinnedToCore(sr_handler_task, "SR Handler Task", 4 * 1024, NULL, configMAX_PRIORITIES - 3, &g_sr_data->handle_task, 0);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio handler task");

    return ESP_OK;
err:
    app_sr_stop();
    return ret;
}

esp_err_t app_sr_stop(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    /**
     * Waiting for all task stoped
     * TODO: A task creation failure cannot be handled correctly now
     * */
    xEventGroupSetBits(g_sr_data->event_group, NEED_DELETE);
    xEventGroupWaitBits(g_sr_data->event_group, NEED_DELETE | FEED_DELETED | DETECT_DELETED, 1, 1, portMAX_DELAY);

    if (g_sr_data->result_que)
    {
        vQueueDelete(g_sr_data->result_que);
        g_sr_data->result_que = NULL;
    }

    if (g_sr_data->event_group)
    {
        vEventGroupDelete(g_sr_data->event_group);
        g_sr_data->event_group = NULL;
    }

    if (g_sr_data->fp)
    {
        fclose(g_sr_data->fp);
        g_sr_data->fp = NULL;
    }

    if (g_sr_data->model_data)
    {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
    }

    if (g_sr_data->afe_data)
    {
        g_sr_data->afe_handle->destroy(g_sr_data->afe_data);
    }

    sr_cmd_t *it;
    while (!SLIST_EMPTY(&g_sr_data->cmd_list))
    {
        it = SLIST_FIRST(&g_sr_data->cmd_list);
        SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
        heap_caps_free(it);
    }

    if (g_sr_data->afe_in_buffer)
    {
        heap_caps_free(g_sr_data->afe_in_buffer);
    }

    if (g_sr_data->afe_out_buffer)
    {
        heap_caps_free(g_sr_data->afe_out_buffer);
    }

    heap_caps_free(g_sr_data);
    g_sr_data = NULL;
    return ESP_OK;
}
