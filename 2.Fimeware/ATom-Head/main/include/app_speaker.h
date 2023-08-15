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

esp_err_t AppSpeaker_Init(void);
size_t speaker_write(char *data, int numData);
void speaker_uninstall(void);

esp_err_t audio_i2s_reconfig_clk(uint32_t rate, uint32_t bits_cfg, i2s_channel_t ch);
esp_err_t audio_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
