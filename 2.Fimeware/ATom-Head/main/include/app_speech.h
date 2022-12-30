#pragma once

#include "driver/gpio.h"

#define ADC_I2S_CHANNEL     (2)
#define FUNC_I2S_EN         (1)
#define GPIO_I2S_LRCK       (GPIO_NUM_42)
#define GPIO_I2S_MCLK       (GPIO_NUM_NC)
#define GPIO_I2S_SCLK       (GPIO_NUM_1)
#define GPIO_I2S_SDIN       (GPIO_NUM_2)
#define GPIO_I2S_DOUT       (GPIO_NUM_NC)

#define I2S_CONFIG_DEFAULT() { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX, \
    .sample_rate            = 16000, \
    .bits_per_sample        = I2S_BITS_PER_SAMPLE_32BIT, \
    .channel_format         = I2S_CHANNEL_FMT_ONLY_LEFT, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL2, \
    .dma_buf_count          = 3, \
    .dma_buf_len            = 300, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = I2S_BITS_PER_CHAN_32BIT, \
}

typedef enum
{
    COMMAND_TIMEOUT = -2,
    COMMAND_NOT_DETECTED = -1,

    MENU_STOP_WORKING = 0,
    MENU_DISPLAY_ONLY = 1,
    MENU_FACE_RECOGNITION = 2,
    MENU_MOTION_DETECTION = 3,

    ACTION_ENROLL = 4,
    ACTION_DELETE = 5,
    ACTION_RECOGNIZE = 6
} command_word_t;

#ifdef __cplusplus
extern "C"
{
#endif

void AppSpeech_Init();
void AppSpeech_run();

#ifdef __cplusplus
}
#endif
