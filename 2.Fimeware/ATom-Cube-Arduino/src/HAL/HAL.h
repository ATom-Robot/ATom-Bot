#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>
#include <Arduino.h>
#include "HAL_Def.h"
#include "Configs/Config.h"
#include "FreeRTOS.h"

namespace HAL
{
void Init();
void Update();

/* Backlight */
void Backlight_Init();
uint32_t Backlight_GetValue();
void Backlight_SetValue(int32_t val);
void Backlight_SetGradual(uint32_t target, uint16_t time = 500);
void Backlight_ForceLit(bool en);

/* I2C */
void I2C_Init(bool startScan);

/* IMU */
void IMU_Init();
void IMU_Update();

/* Buzzer */
void Buzz_init();
void Buzz_SetEnable(bool en);
void Buzz_Tone(uint32_t freq, int32_t duration = 0);

/* Led */
void Led_Init();
void setRGB(int id, int r, int g, int b);
void setBrightness(float duty);
void Led_firt_light(void);
void led_rainbow(void);

/* Audio */
void Audio_Init();
void Audio_Update();
bool Audio_PlayMusic(const char *name);
}

#endif
