#pragma once

#include "driver/gpio.h"

#define SAMPLE_RATE (16000)
#define I2S_NUM     I2S_NUM_0

#define I2S_DOUT    GPIO_NUM_46
#define I2S_BCLK    GPIO_NUM_3
#define I2S_LRC     GPIO_NUM_20

#ifdef __cplusplus
extern "C"
{
#endif

void speaker_init(void);

size_t speaker_write(char *data, int numData);

void speaker_uninstall(void);

#ifdef __cplusplus
}
#endif
