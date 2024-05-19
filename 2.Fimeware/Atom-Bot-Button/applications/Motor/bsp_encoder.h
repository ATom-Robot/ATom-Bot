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

// 电机转一圈单相输出7个脉冲，1:100减速比，电机输出轴转一圈最大输出(100*7*4) 2800个计数
// 轮子周长 L = (d=1.68cm * pi) = 5.278cm
// AB相输出脉冲信号相位差为90°，可检测电机转动方向
#define ENCODER_CNT_PER_ROUND       (2800.0)
#define WHEEL_CIRCUMFERENCE_CM      (5.278)
#define ENCODER_CNT_10MS_2_SPD_MM_S (100.0 * WHEEL_CIRCUMFERENCE_CM * 10 / ENCODER_CNT_PER_ROUND)

void Encoder_Init(void);
void Encoder_Set_Counter(int8_t Motor_Num, int16_t count);

int32_t Encoder_Get_Counter(uint8_t Motor_Num);
float Motor_Speed(int encoder_cnt, uint16_t ppr, uint16_t ratio, uint16_t cnt_time);
int32_t Num_Encoder_Cnt(float num, uint16_t ppr, uint16_t ratio);
int32_t Rpm_Encoder_Cnt(float rpm, uint16_t ppr, uint16_t ratio, uint16_t cnt_time);

#endif
