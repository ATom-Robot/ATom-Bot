#pragma once

#include "driver/gpio.h"
#include "driver/i2s.h"

#define ADUIO_SAMPLE_RATE   (44100)
#define ADUIO_SAMPLE_BITS   (16)

#define I2S_NUM     I2S_NUM_0

#define I2S_DOUT    GPIO_NUM_46
#define I2S_BCLK    GPIO_NUM_3
#define I2S_LRC     GPIO_NUM_20

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t App_Speaker_Init(void);
esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);
esp_err_t bsp_codec_set_fn(uint32_t rate, uint32_t bits_cfg, i2s_channel_t ch);
void speaker_uninstall(void);

#ifdef __cplusplus
}
#endif
