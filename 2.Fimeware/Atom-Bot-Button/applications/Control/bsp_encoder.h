/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <rtthread.h>

#define ENCODER_ID_A        3
#define ENCODER_ID_B        2

void Encoder_Init(void);
void Encoder_Set_Counter(int8_t Motor_Num, int16_t count);

int32_t Encoder_Get_Counter(uint8_t Motor_Num);
long Num_Encoder_Cnt(float num, uint16_t ppr, uint16_t ratio);
long Rpm_Encoder_Cnt(float rpm, uint16_t ppr, uint16_t ratio, uint16_t cnt_time);
float Motor_Speed(int encoder_cnt, uint16_t ppr, uint16_t ratio, uint16_t cnt_time);

#endif
