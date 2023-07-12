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

#define ENCODER_MID_VALUE   0x7fff

void Encoder_Init(void);
void Encoder_Set_Counter(int8_t Motor_Num, int16_t count);

int Encoder_Get_Count_Now(uint8_t Encoder_id);
void Encoder_Update_Count(uint8_t Encoder_id);

#endif
