/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <rtthread.h>

#define PWMA1   TIM4->CCR2
#define PWMA2   TIM4->CCR1

#define PWMB1   TIM4->CCR3
#define PWMB2   TIM4->CCR4

#define MOTOR_ID_1     1
#define MOTOR_ID_2     2

#define	MOTOR_MAX_PULSE	3550
#define	Dead_Voltage	2070     /* 死区电压 */
#define	Rpm_Max			250      /* 最大转速 */

// 金属输出轴编码器电机：
// 电机转一圈单相输出7个脉冲，1:150减速比，电机输出轴转一圈最大输出(150*7*4) 4200个计数
// AB相输出脉冲信号相位差为90°，可检测电机转动方向
#define ENCODER_CNT_PER_ROUND       (4200.0)
#define WHEEL_CIRCUMFERENCE_CM      (6.40)
#define ENCODER_CNT_10MS_2_SPD_MM_S (100.0 * WHEEL_CIRCUMFERENCE_CM * 10 / ENCODER_CNT_PER_ROUND)

void Motor_Init(void);
void Motor_Set_Pwm(uint8_t id, int speed);
void Get_Motor_Speed(int *leftSpeed, int *rightSpeed);

#endif

