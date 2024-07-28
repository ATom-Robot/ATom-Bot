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

#define MOTOR_MAX_PULSE 3560
#define Dead_Voltage    2100     /* 死区电压 */
#define RPM_MAX         300      /* 最大转速 */

void Motor_Init(void);
void Motor_Set_Pwm(uint8_t id, int speed);
void Get_Motor_Speed(int *leftSpeed, int *rightSpeed);

#endif
