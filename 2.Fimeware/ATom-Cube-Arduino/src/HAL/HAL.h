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

/* Power */
void Power_Init();
void Power_HandleTimeUpdate();
void Power_SetAutoLowPowerTimeout(uint16_t sec);
uint16_t Power_GetAutoLowPowerTimeout();
void Power_SetAutoLowPowerEnable(bool en);
void Power_Shutdown();
void Power_Update();
void Power_EventMonitor();
void Power_GetInfo(Power_Info_t *info);
typedef void(* Power_CallbackFunction_t)(void);
void Power_SetEventCallback(Power_CallbackFunction_t callback);

/* Buzzer */
void Buzz_init();
void Buzz_SetEnable(bool en);
void Buzz_Tone(uint32_t freq, int32_t duration = 0);

/* Audio */
void Audio_Init();
void Audio_Update();
bool Audio_PlayMusic(const char *name);
}

#endif
