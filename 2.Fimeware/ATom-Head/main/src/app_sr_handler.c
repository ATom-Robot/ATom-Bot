/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "app_speech.h"
#include "file_manager.h"
#include "app_player.h"
#include "app_uart.h"
#include "app_ui.h"
#include "driver/i2s.h"
#include "app_sr_handler.h"

#define SR_RUN_TEST 1
#define SR_CONTINUE_DET 0

static const char *TAG = "APP/sr_handler";

static bool b_audio_playing = false;

/* 距离变量 */
static int8_t distance = 0;

typedef enum
{
    AUDIO_WAKE,
    AUDIO_OK,
    AUDIO_END,
    AUDIO_MAX,
} audio_segment_t;

typedef enum
{
    TURN_AROUND_T,
    MOVE_FORWARD_T,
    MOVE_BACKWARD_T,
} TimerX;

typedef struct
{
    uint8_t *audio_buffer;
    size_t len;
} audio_data_t;

static audio_data_t g_audio_data[AUDIO_MAX];

static esp_err_t sr_echo_play(audio_segment_t audio)
{
    typedef struct
    {
        // The "RIFF" chunk descriptor
        uint8_t ChunkID[4];
        int32_t ChunkSize;
        uint8_t Format[4];
        // The "fmt" sub-chunk
        uint8_t Subchunk1ID[4];
        int32_t Subchunk1Size;
        int16_t AudioFormat;
        int16_t NumChannels;
        int32_t SampleRate;
        int32_t ByteRate;
        int16_t BlockAlign;
        int16_t BitsPerSample;
        // The "data" sub-chunk
        uint8_t Subchunk2ID[4];
        int32_t Subchunk2Size;
    } wav_header_t;

    /**
     * read head of WAV file
     */
    uint8_t *p = g_audio_data[audio].audio_buffer;
    wav_header_t *wav_head = (wav_header_t *)p;
    if (NULL == strstr((char *)wav_head->Subchunk1ID, "fmt") &&
        NULL == strstr((char *)wav_head->Subchunk2ID, "data"))
    {
        ESP_LOGE(TAG, "Header of wav format error");
        return ESP_FAIL;
    }
    p += sizeof(wav_header_t);
    size_t len = g_audio_data[audio].len - sizeof(wav_header_t);
    len = len & 0xfffffffc;
    ESP_LOGD(TAG, "frame_rate=%d, ch=%d, width=%d", wav_head->SampleRate, wav_head->NumChannels, wav_head->BitsPerSample);

    i2s_zero_dma_buffer(I2S_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(50));

    size_t bytes_written = 0;
    b_audio_playing = true;
    i2s_write(I2S_NUM_0, p, len, &bytes_written, portMAX_DELAY);
    i2s_zero_dma_buffer(I2S_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(20));
    b_audio_playing = false;
    return ESP_OK;
}

bool sr_echo_is_playing(void)
{
    return b_audio_playing;
}

/* 定时器句柄 */
TimerHandle_t xOneShotTimer;

/* 定时器回调函数 */
void vTimerCallback(TimerHandle_t xTimer)
{
    /* 串口发送数据 */
    sendwl_ChassisSpeedData(0, 0);
}

// Fisher-Yates洗牌
void shuffle(int array[], int size)
{
    for (int i = size - 1; i > 0; --i)
    {
        int j = rand() % (i + 1);
        int temp = array[i];
        array[i] = array[j];
        array[j] = temp;
    }
}

void sr_handler_task(void *pvParam)
{
#if !SR_RUN_TEST
    esp_err_t ret;

    FILE *fp;
    // const sys_param_t *param = settings_get_parameter();
    const char *files[1][3] =
        {
            {"/spiffs/wakeup.wav", "/spiffs/wakeup.wav", "/spiffs/wakeup.wav"},
        };
    char audio_file[48] = {0};
    for (size_t i = 0; i < AUDIO_MAX; i++)
    {
        strncpy(audio_file, files[0][i], sizeof(audio_file));
        fp = fopen(audio_file, "rb");
        ESP_GOTO_ON_FALSE(NULL != fp, ESP_ERR_NOT_FOUND, err, TAG, "Open file %s failed", audio_file);
        size_t file_size = fm_get_file_size(audio_file);
        g_audio_data[i].len = file_size;
        g_audio_data[i].audio_buffer = heap_caps_calloc(1, file_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        ESP_GOTO_ON_FALSE(NULL != g_audio_data[i].audio_buffer, ESP_ERR_NO_MEM, err, TAG, "No mem for sr echo buffer");
        fread(g_audio_data[i].audio_buffer, 1, file_size, fp);
        fclose(fp);
    }
    (void)ret;
#endif

    // player_state_t last_player_state = PLAYER_STATE_IDLE;
    audio_player_state_t last_player_state = AUDIO_PLAYER_STATE_IDLE;

    // 初始化随机数生成器
    srand(time(NULL));

    // 创建索引数组并初始化为顺序索引
    int indices[SONG_COUNT];
    for (int i = 0; i < SONG_COUNT; ++i)
    {
        indices[i] = i;
    }

    while (true)
    {
        sr_result_t result;
        app_sr_get_result(&result, portMAX_DELAY);
        char audio_file[48] = {0};

        // Time out
        if (ESP_MN_STATE_TIMEOUT == result.state)
        {
#if !SR_RUN_TEST
            sr_echo_play(AUDIO_END);
#endif
            ui_wakeup_emoji_over();
            continue;
        }

        // Detected
        if (AFE_FETCH_WWE_DETECTED == result.fetch_mode)
        {
            ESP_LOGI(TAG, "WAKE UP!");

            ui_wakeup_emoji_start();
            audio_player_play_name("listen.mp3");

            last_player_state = audio_player_get_state();
            // app_player_pause();

#if !SR_RUN_TEST
            sr_echo_play(AUDIO_WAKE);
#endif
            continue;
        }

        // Parse word
        if (result.state != ESP_MN_STATE_DETECTING && result.state != ESP_MN_STATE_TIMEOUT)
        {
            const sr_cmd_t *cmd = app_sr_get_cmd_from_id(result.command_id);
            ESP_LOGI(TAG, "command:%s, act:%d", cmd->str, cmd->cmd);

            ui_wakeup_emoji_over();

            switch (cmd->cmd)
            {
            case SR_CMD_SET_RED:
                break;
            // 下一首
            case SR_CMD_PLAY_NEXT:
                break;
            // 上一首
            case SR_CMD_PLAY_PREV:
                break;
            case SR_CMD_SING:
            case SR_CMD_MUSIC:
            {
                // 随机打乱索引数组
                shuffle(indices, SONG_COUNT);
                audio_player_play_name(song_list[indices[0]]);
                ui_sr_emoji_display(SING2_EMOJI, false);
                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            case SR_CMD_PLAY_HAPPY:
            {
                audio_player_play_name("sing2.mp3");
                ui_sr_emoji_display(HAPPY_EMOJI, false);
                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            case SR_CMD_PLAY_SCARE:
            {
                audio_player_play_name("scared.mp3");
                ui_sr_emoji_display(SCARED_EMOJI, false);
                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            case SR_CMD_PLAY_POLICE:
            {
                audio_player_play_name("police.mp3");
                ui_sr_emoji_display(SHAKE_EMOJI, false);
                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            /* 跳舞 */
            case SR_CMD_PLAY_DANCE:
            {
                /* to do */
                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            /* 转圈 */
            case SR_CMD_PLAY_CIRCLE:
            {
                /* 串口发送数据 -- 4s */
                audio_player_play_name("sing2.mp3");
                ui_sr_emoji_display(HAPPY_EMOJI, false);
                sendwl_ChassisSpeedData(0, -40);

                /* 创建单次定时器 */
                xOneShotTimer = xTimerCreate(
                    "OneShotTimer",        /* 定时器名称 */
                    pdMS_TO_TICKS(4000),   /* 定时器周期 */
                    pdFALSE,               /* 单次定时器 */
                    (void *)TURN_AROUND_T, /* 定时器 ID */
                    vTimerCallback         /* 定时器回调函数 */
                );
                xTimerStart(xOneShotTimer, 0);

                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            /* 向前进 */
            case SR_CMD_PLAY_MOVE_FORWARD:
            {
                // distance += 4;
                // /* 串口发送数据 -- 向前走4s */
                // sendwl_Chassis_DistanceData(distance, 0, 50, 50);
                sendwl_ChassisSpeedData(45, 0);

                /* 创建单次定时器 */
                xOneShotTimer = xTimerCreate(
                    "OneShotTimer",         /* 定时器名称 */
                    pdMS_TO_TICKS(3000),    /* 定时器周期 */
                    pdFALSE,                /* 单次定时器 */
                    (void *)MOVE_FORWARD_T, /* 定时器 ID */
                    vTimerCallback          /* 定时器回调函数 */
                );
                xTimerStart(xOneShotTimer, 0);

                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            /* 向后退 */
            case SR_CMD_PLAY_MOVE_BACKWARD:
            {
                // distance -= 4;
                audio_player_play_name("bi.mp3");
                /* 串口发送数据 -- 向后退5s */
                // sendwl_Chassis_DistanceData(distance, 0, 50, 50);
                sendwl_ChassisSpeedData(-45, 0);

                /* 创建单次定时器 */
                xOneShotTimer = xTimerCreate(
                    "OneShotTimer",          /* 定时器名称 */
                    pdMS_TO_TICKS(3000),     /* 定时器周期 */
                    pdFALSE,                 /* 单次定时器 */
                    (void *)MOVE_BACKWARD_T, /* 定时器 ID */
                    vTimerCallback           /* 定时器回调函数 */
                );
                xTimerStart(xOneShotTimer, 0);

                last_player_state = AUDIO_PLAYER_STATE_PLAYING;
                break;
            }
            // 暂停
            case SR_CMD_PLAY_STOP:
            case SR_CMD_PLAY_PAUSE:
                audio_player_pause();
                last_player_state = AUDIO_PLAYER_STATE_PAUSE;
                break;
            default:
                ESP_LOGE(TAG, "Unknow cmd");
                break;
            }
#if !SR_RUN_TEST
            strncpy(audio_file, "/spiffs/echo_cn_ok.wav", sizeof(audio_file));
            sr_echo_play(AUDIO_OK);
#endif
        }
    }
#if !SR_RUN_TEST
err:
    if (fp)
    {
        fclose(fp);
    }
    vTaskDelete(NULL);
#endif
}
