#pragma once

#include <stdbool.h>
#include <sys/queue.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_idf_version.h"
#include "esp_err.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_models.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ADC_I2S_CHANNEL     (2)
#define FUNC_I2S_EN         (1)
#define GPIO_I2S_LRCK       (GPIO_NUM_42)
#define GPIO_I2S_MCLK       (GPIO_NUM_NC)
#define GPIO_I2S_SCLK       (GPIO_NUM_1)
#define GPIO_I2S_SDIN       (GPIO_NUM_2)
#define GPIO_I2S_DOUT       (GPIO_NUM_NC)

#define SR_CMD_STR_LEN_MAX 64
#define SR_CMD_PHONEME_LEN_MAX 64

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate), \
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_chan, channel_fmt), \
        .gpio_cfg = { \
            .mclk = GPIO_I2S_MCLK, \
            .bclk = GPIO_I2S_SCLK, \
            .ws   = GPIO_I2S_LRCK, \
            .dout = GPIO_I2S_DOUT, \
            .din  = GPIO_I2S_SDIN, \
            .invert_flags = { \
                .mclk_inv = false, \
                .bclk_inv = false, \
                .ws_inv   = false, \
            }, \
        }, \
    }
#else
#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_RX, \
    .sample_rate            = 16000, \
    .bits_per_sample        = I2S_BITS_PER_SAMPLE_32BIT, \
    .channel_format         = I2S_CHANNEL_FMT_ONLY_LEFT, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1, \
    .dma_buf_count          = 4, \
    .dma_buf_len            = 256, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = I2S_BITS_PER_CHAN_32BIT, \
}
#endif

typedef enum
{
    SR_LANG_EN,
    SR_LANG_CN,
    SR_LANG_MAX,
} sr_language_t;

typedef enum
{
    SR_CMD_SET_RED = 0,
    SR_CMD_SING,
    SR_CMD_MUSIC,
    SR_CMD_PLAY_NEXT,
    SR_CMD_PLAY_PREV,
    SR_CMD_PLAY_PAUSE,
    SR_CMD_PLAY_STOP,
    SR_CMD_PLAY_POLICE,
    SR_CMD_PLAY_SCARE,
    SR_CMD_PLAY_HAPPY,
    SR_CMD_PLAY_CIRCLE,
    SR_CMD_PLAY_DANCE,
    SR_CMD_PLAY_MOVE_FORWARD,
    SR_CMD_PLAY_MOVE_BACKWARD,
    SR_CMD_ENTER_AI_MODE,
    SR_CMD_EXIT_AI_MODE,
    SR_CMD_MAX,
} sr_user_cmd_t;

// typedef enum
// {
//     ESP_MN_STATE_DETECTING = -1,     // detecting
//     ESP_MN_STATE_TIMEOUT = -2,       // time out
//     ESP_MN_STATE_DETECTED = -3,      // detected
// } esp_mn_state_t;

typedef struct sr_cmd_t
{
    sr_user_cmd_t cmd;
    sr_language_t lang;
    uint32_t id;
    char str[SR_CMD_STR_LEN_MAX];
    char phoneme[SR_CMD_PHONEME_LEN_MAX];
    SLIST_ENTRY(sr_cmd_t) next;
} sr_cmd_t;

typedef struct
{
    wakenet_state_t wakenet_mode;
    esp_mn_state_t state;
    int command_id;
} sr_result_t;

esp_err_t App_Speech_Init(void);
esp_err_t App_Speech_run(void);
esp_err_t app_sr_start(bool record_en);
esp_err_t app_sr_stop(void);
esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait);
esp_err_t app_sr_set_language(sr_language_t new_lang);
esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd);
esp_err_t app_sr_modify_cmd(uint32_t id, const sr_cmd_t *cmd);
esp_err_t app_sr_remove_cmd(uint32_t id);
esp_err_t app_sr_remove_all_cmd(void);
const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id);
uint8_t app_sr_search_cmd_from_user_cmd(sr_user_cmd_t user_cmd, uint8_t *id_list, uint16_t max_len);
uint8_t app_sr_search_cmd_from_phoneme(const char *phoneme, uint8_t *id_list, uint16_t max_len);
esp_err_t app_sr_update_cmds(void);

#ifdef __cplusplus
}
#endif
