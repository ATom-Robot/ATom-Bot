#pragma once

#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Led */
void Led_Init(void);
void setRGB(int id, int r, int g, int b);
void setBrightness(float duty);
void Led_firt_light(void);
void led_rainbow(void);

#ifdef __cplusplus
}
#endif
