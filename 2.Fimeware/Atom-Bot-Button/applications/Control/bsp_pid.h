/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#ifndef BSP_PID_H
#define BSP_PID_H

#include <rtthread.h>

// left wheel pid
#define PID_KP_DEF_L             (0.2)
#define PID_KI_DEF_L             (0.0)
#define PID_KD_DEF_L             (0.0)

// right wheel pid
#define PID_KP_DEF_R             (0.1)
#define PID_KI_DEF_R             (0.0)
#define PID_KD_DEF_R             (0.0)

struct pid_uint
{
    float Kp;				//比例
    float Ki;				//积分
    float Kd;				//微分

	float Bias;
	float Integral_bias;
    float Last_bias;		//调节量
    float Prev_bias;		//调节量

    int16_t target;			//速度设置
    int16_t reality;		//当前速度
};

extern struct pid_uint pid_wheel_Left;
extern struct pid_uint pid_wheel_Right;

void PID_Init(void);
int limit_amplitude(int data, int max);
int Position_PID(struct pid_uint *pid);
int Incremental_PID(struct pid_uint *pid);

#endif
