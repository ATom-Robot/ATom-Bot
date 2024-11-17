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
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "model_path.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "model_path.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "app_speech.h"
#include "app_player.h"
#include "app_sr_handler.h"

#define NEED_DELETE BIT0
#define FEED_DELETED BIT1
#define DETECT_DELETED BIT2

static const char *TAG = "App/Speech";
typedef struct
{
    sr_language_t lang;
    char *mn_name;
    model_iface_data_t *model_data;
    const esp_mn_iface_t *multinet;
    const esp_afe_sr_iface_t *afe_handle;
    esp_afe_sr_data_t *afe_data;
    int16_t *afe_in_buffer;
    int16_t *afe_out_buffer;
    SLIST_HEAD(sr_cmd_list_t, sr_cmd_t) cmd_list;
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
static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static SemaphoreHandle_t sr_detect_semaphore = NULL;

static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    i2s_config_t i2s_config = I2S_CONFIG_DEFAULT(sample_rate, I2S_CHANNEL_FMT_ONLY_LEFT, bits_per_chan);
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ret = i2s_channel_read(rx_handle, buffer, buffer_len, &bytes_read, portMAX_DELAY);
#else
    ret = i2s_read(I2S_NUM_1, buffer, buffer_len, &bytes_read, portMAX_DELAY);
#endif

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
    {SR_CMD_SING, SR_LANG_CN, 0, "唱首歌", "chang shou ge", {NULL}},
    {SR_CMD_MUSIC, SR_LANG_CN, 0, "音乐", "yin yue", {NULL}},
    {SR_CMD_PLAY_NEXT, SR_LANG_CN, 0, "下一首", "xia yi shou", {NULL}},
    {SR_CMD_PLAY_PREV, SR_LANG_CN, 0, "上一首", "shang yi shou", {NULL}},
    {SR_CMD_PLAY_PAUSE, SR_LANG_CN, 0, "暂停", "zan ting", {NULL}},
    {SR_CMD_PLAY_STOP, SR_LANG_CN, 0, "停止", "ting zhi", {NULL}},
    {SR_CMD_PLAY_POLICE, SR_LANG_CN, 0, "警车", "jing che", {NULL}},
    {SR_CMD_PLAY_SCARE, SR_LANG_CN, 0, "害怕", "hai pa", {NULL}},
    {SR_CMD_PLAY_HAPPY, SR_LANG_CN, 0, "开心", "kai xin", {NULL}},
    {SR_CMD_PLAY_HAPPY, SR_LANG_CN, 0, "高兴", "gao xing", {NULL}},
    {SR_CMD_PLAY_CIRCLE, SR_LANG_CN, 0, "转圈", "zhuan quan", {NULL}},
    {SR_CMD_PLAY_DANCE, SR_LANG_CN, 0, "跳舞", "tiao wu", {NULL}},
    {SR_CMD_PLAY_MOVE_FORWARD, SR_LANG_CN, 0, "向前", "xiang qian", {NULL}},
    {SR_CMD_PLAY_MOVE_BACKWARD, SR_LANG_CN, 0, "向后", "xiang hou", {NULL}},
    {SR_CMD_ENTER_AI_MODE, SR_LANG_CN, 0, "开启聊天", "kai qi liao tian", {NULL}},
    {SR_CMD_EXIT_AI_MODE, SR_LANG_CN, 0, "关闭聊天", "guan bi liao tian", {NULL}},
};

static void feed_Task(void *pvParam)
{
    size_t bytes_read = 0;

    ESP_LOGI(TAG, "Feed Task");
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) pvParam;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = bsp_get_feed_channel();
    assert(nch <= feed_channel);
    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);

    /* Allocate audio buffer and check for result */
    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(audio_buffer);
    g_sr_data->afe_in_buffer = audio_buffer;

    while (1)
    {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group))
        {
            xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
            vTaskDelete(NULL);
        }

        /* Read audio data from I2S bus */
        bsp_get_feed_data(audio_buffer, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, (int16_t *)audio_buffer);

        // 唤醒后才开始录音 --> AI大模型使用
        audio_record_save(audio_buffer, audio_chunksize);
    }
    if (audio_buffer)
    {
        heap_caps_free(audio_buffer);
        audio_buffer = NULL;
    }
    vTaskDelete(NULL);
}

static void audio_detect_task(void *pvParam)
{
    bool detect_flag = false;
    static uint8_t frame_keep = 0;
    static afe_vad_state_t local_state;
    esp_afe_sr_data_t *afe_data = pvParam;

    // if (xSemaphoreTake(sr_detect_semaphore, portMAX_DELAY) == pdTRUE)
    // {
    //     vSemaphoreDelete(sr_detect_semaphore);
    // }
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int mu_chunksize = g_sr_data->multinet->get_samp_chunksize(g_sr_data->model_data);
    assert(mu_chunksize == afe_chunksize);

    ESP_LOGI(TAG, "------------detect start------------\n");

    while (1)
    {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group))
        {
            xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
            vTaskDelete(g_sr_data->handle_task);
            vTaskDelete(NULL);
        }

        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL)
        {
            ESP_LOGW(TAG, "AFE Fetch Fail");
            continue;
        }

        // 检测到唤醒词
        if (WAKENET_DETECTED == res->wakeup_state)
        {
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) "wakeword detected");
            sr_result_t result =
            {
                .wakenet_mode = WAKENET_DETECTED,
                .state = ESP_MN_STATE_DETECTING,
                .command_id = 0,
            };
            xQueueSend(g_sr_data->result_que, &result, 0);
        }
        else if (WAKENET_CHANNEL_VERIFIED == res->wakeup_state)
        {
            detect_flag = true;
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) ">>> Say your command <<<");

            frame_keep = 0;
            // 关闭唤醒词检测
            g_sr_data->afe_handle->disable_wakenet(afe_data);
        }

        if (true == detect_flag)
        {
            // if (AFE_VAD_SPEECH == res->vad_state)
            // {
            //     frame_keep = 0;
            // }
            // else
            // {
            //     frame_keep++;
            // }
            // if ((100 == frame_keep) && (AFE_VAD_SILENCE == res->vad_state))
            // {
            //     ESP_LOGW(TAG, "speak over");
            //     sr_result_t result =
            //     {
            //         .wakenet_mode = WAKENET_NO_DETECT,
            //         .state = ESP_MN_STATE_TIMEOUT,
            //         .command_id = 0,
            //     };
            //     xQueueSend(g_sr_data->result_que, &result, 0);
            //     g_sr_data->afe_handle->enable_wakenet(afe_data);
            //     detect_flag = false;
            //     // continue;
            // }

            esp_mn_state_t mn_state = ESP_MN_STATE_DETECTING;
            // 开始离线语音识别
            mn_state = g_sr_data->multinet->detect(g_sr_data->model_data, res->data);

            if (ESP_MN_STATE_DETECTING == mn_state)
                continue;

            if (ESP_MN_STATE_TIMEOUT == mn_state)
            {
                ESP_LOGW(TAG, "Time out");
                sr_result_t result =
                {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = 0,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }

            if (ESP_MN_STATE_DETECTED == mn_state)
            {
                esp_mn_results_t *mn_result = g_sr_data->multinet->get_results(g_sr_data->model_data);
                for (int i = 0; i < mn_result->num; i++)
                {
                    printf("TOP %d, command_id: %d, phrase_id: %d, prob: %f\n",
                           i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->prob[i]);
                }

                int sr_command_id = mn_result->command_id[0];
                ESP_LOGI(TAG, "Deteted command : %d", sr_command_id);
                sr_result_t result =
                {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = sr_command_id,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
#if !SR_CONTINUE_DET
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
#endif
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

esp_err_t App_Speech_Init(void)
{
    esp_err_t res = ESP_OK;
    res = bsp_i2s_init(I2S_NUM_1, 16000, 2, 32);
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

esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG, "pointer of cmd is invaild");
    ESP_RETURN_ON_FALSE(ESP_MN_MAX_PHRASE_NUM >= g_sr_data->cmd_num, ESP_ERR_INVALID_STATE, TAG, "cmd is full");

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
#else  // insert head
    SLIST_INSERT_HEAD(&g_sr_data->cmd_list, it, next);
#endif

    if (strstr(g_sr_data->mn_name, "mn6_en"))
    {
        esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->str);
    }
    else
    {
        esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->phoneme);
    }
    g_sr_data->cmd_num++;
    return ESP_OK;
}

esp_err_t app_sr_modify_cmd(uint32_t id, const sr_cmd_t *cmd)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG, "pointer of cmd is invaild");
    ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, ESP_ERR_INVALID_ARG, TAG, "cmd id out of range");
    ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG, "cmd lang error");

    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        if (it->id == id)
        {
            ESP_LOGI(TAG, "modify cmd [%d] from %s to %s", id, it->str, cmd->str);
            if (strstr(g_sr_data->mn_name, "mn6_en"))
            {
                esp_mn_commands_modify(it->str, (char *)cmd->str);
            }
            else
            {
                esp_mn_commands_modify(it->phoneme, (char *)cmd->phoneme);
            }
            memcpy(it, cmd, sizeof(sr_cmd_t));
            break;
        }
    }
    ESP_RETURN_ON_FALSE(NULL != it, ESP_ERR_NOT_FOUND, TAG, "can't find cmd id:%d", cmd->id);
    return ESP_OK;
}

esp_err_t app_sr_remove_cmd(uint32_t id)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, ESP_ERR_INVALID_ARG, TAG, "cmd id out of range");
    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        if (it->id == id)
        {
            ESP_LOGI(TAG, "remove cmd id [%d]", it->id);
            SLIST_REMOVE(&g_sr_data->cmd_list, it, sr_cmd_t, next);
            heap_caps_free(it);
            g_sr_data->cmd_num--;
            break;
        }
    }
    ESP_RETURN_ON_FALSE(NULL != it, ESP_ERR_NOT_FOUND, TAG, "can't find cmd id:%d", id);
    return ESP_OK;
}

esp_err_t app_sr_remove_all_cmd(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    sr_cmd_t *it;
    while (!SLIST_EMPTY(&g_sr_data->cmd_list))
    {
        it = SLIST_FIRST(&g_sr_data->cmd_list);
        SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
        heap_caps_free(it);
    }
    SLIST_INIT(&g_sr_data->cmd_list);
    return ESP_OK;
}

esp_err_t app_sr_update_cmds(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    uint32_t count = 0;
    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        it->id = count++;
    }

    esp_mn_error_t *err_id = esp_mn_commands_update(g_sr_data->multinet, g_sr_data->model_data);
    if (err_id)
    {
        for (int i = 0; i < err_id->num; i++)
        {
            ESP_LOGE(TAG, "err cmd id:%d", err_id->phrases[i]->command_id);
        }
    }
    esp_mn_commands_print();

    return ESP_OK;
}

uint8_t app_sr_search_cmd_from_user_cmd(sr_user_cmd_t user_cmd, uint8_t *id_list, uint16_t max_len)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, 0, TAG, "SR is not running");

    uint8_t cmd_num = 0;
    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        if (user_cmd == it->cmd)
        {
            if (id_list)
            {
                id_list[cmd_num] = it->id;
            }
            if (++cmd_num >= max_len)
            {
                break;
            }
        }
    }
    return cmd_num;
}

uint8_t app_sr_search_cmd_from_phoneme(const char *phoneme, uint8_t *id_list, uint16_t max_len)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, 0, TAG, "SR is not running");

    uint8_t cmd_num = 0;
    sr_cmd_t *it;
    SLIST_FOREACH(it, &g_sr_data->cmd_list, next)
    {
        if (0 == strcmp(phoneme, it->phoneme))
        {
            if (id_list)
            {
                id_list[cmd_num] = it->id;
            }
            if (++cmd_num >= max_len)
            {
                break;
            }
        }
    }
    return cmd_num;
}

esp_err_t app_sr_set_language(sr_language_t new_lang)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    g_sr_data->cmd_num = 0;

    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "wn9_alexa");
    ESP_RETURN_ON_FALSE(NULL != wn_name, ESP_ERR_INVALID_ARG, TAG, "Modifications to the code are required to support the relevant configuration");
    g_sr_data->afe_handle->set_wakenet(g_sr_data->afe_data, wn_name);
    ESP_LOGI(TAG, "load wakenet:%s", wn_name);

    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    ESP_RETURN_ON_FALSE(NULL != mn_name, ESP_ERR_INVALID_ARG, TAG, "Modifications to the code are required to support the relevant configuration");
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 5760);
    g_sr_data->multinet = multinet;
    g_sr_data->model_data = model_data;
    g_sr_data->mn_name = mn_name;
    ESP_LOGI(TAG, "load multinet:%s", g_sr_data->mn_name);

    // remove all command
    app_sr_remove_all_cmd();
    if (strstr(g_sr_data->mn_name, "mn6"))
    {
        esp_mn_commands_clear();
    }

    uint8_t cmd_number = 0;
    // count command number
    for (size_t i = 0; i < sizeof(g_default_cmd_info) / sizeof(sr_cmd_t); i++)
    {
        app_sr_add_cmd(&g_default_cmd_info[i]);
        cmd_number++;
    }
    ESP_LOGI(TAG, "cmd_number=%d", cmd_number);

    return app_sr_update_cmds();/* Reset command list */
}

void en_sr_detect_task(void)
{
    if (sr_detect_semaphore)
        xSemaphoreGive(sr_detect_semaphore);
}

esp_err_t App_Speech_run(void)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(NULL == g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR already running");

    g_sr_data = heap_caps_calloc(1, sizeof(sr_data_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_NO_MEM, TAG, "Failed create sr data");

    g_sr_data->result_que = xQueueCreate(3, sizeof(sr_result_t));
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG, "Failed create result queue");

    g_sr_data->event_group = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG, "Failed create event_group");

    SLIST_INIT(&g_sr_data->cmd_list);

    models = esp_srmodel_init("model");

    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config.wakenet_init = true;
    afe_config.voice_communication_init = false;
    afe_config.aec_init = false;

    afe_config.pcm_config.total_ch_num = 2;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 1;
    afe_config.wakenet_mode = DET_MODE_90;
    afe_config.se_init = false;

    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    g_sr_data->afe_handle = afe_handle;
    g_sr_data->afe_data = afe_data;
    g_sr_data->lang = SR_LANG_MAX;
    ret = app_sr_set_language(SR_LANG_EN);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ESP_FAIL, err, TAG, "Failed to set language");

    BaseType_t ret_val = xTaskCreatePinnedToCore((TaskFunction_t)feed_Task, "App/SR/Feed", 8 * 1024, afe_data, 5, &g_sr_data->feed_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG, "Failed create audio feed task");

    ret_val = xTaskCreatePinnedToCore((TaskFunction_t)audio_detect_task, "App/SR/Detect", 6 * 1024, afe_data, 5, &g_sr_data->detect_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG, "Failed create audio detect task");

    ret_val = xTaskCreatePinnedToCore(sr_handler_task, "SR Handler Task", 4 * 1024, NULL, configMAX_PRIORITIES - 3, &g_sr_data->handle_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG, "Failed create audio handler task");

    sr_detect_semaphore = xSemaphoreCreateBinary();

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
