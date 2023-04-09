#pragma once

#include <stdbool.h>
#include <sys/queue.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_afe_sr_models.h"
// #include "esp_mn_models.h"
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

#define I2S_CONFIG_DEFAULT() { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX, \
    .sample_rate            = 16000, \
    .bits_per_sample        = I2S_BITS_PER_SAMPLE_32BIT, \
    .channel_format         = I2S_CHANNEL_FMT_ONLY_LEFT, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1, \
    .dma_buf_count          = 6, \
    .dma_buf_len            = 160, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = I2S_BITS_PER_CHAN_32BIT, \
}

typedef enum
{
    SR_LANG_EN,
    SR_LANG_CN,
    SR_LANG_MAX,
} sr_language_t;

typedef enum
{
    SR_CMD_SET_RED = 0,
    SR_CMD_NEXT,
    SR_CMD_PLAY,
    SR_CMD_PAUSE,
    SR_CMD_MAX,
} sr_user_cmd_t;

typedef enum
{
    ESP_MN_STATE_DETECTING = -1,     // detecting
    ESP_MN_STATE_TIMEOUT = -2,       // time out
} esp_mn_state_t;

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
    afe_fetch_mode_t fetch_mode;
    esp_mn_state_t state;
    int command_id;
} sr_result_t;

void AppSpeech_Init();
esp_err_t AppSpeech_run(void);
esp_err_t app_sr_stop(void);
const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id);
esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait);

#ifdef __cplusplus
}
#endif
