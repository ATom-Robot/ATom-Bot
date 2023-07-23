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
#define PID_KP_POS_L             (0.001)
#define PID_KI_POS_L             (0.0)
#define PID_KD_POS_L             (0.0045)

#define PID_KP_VEL_L             (0.02)
#define PID_KI_VEL_L             (0.0)
#define PID_KD_VEL_L             (0.0)

// right wheel pid
#define PID_KP_POS_R             (0.0)
#define PID_KI_POS_R             (0.0)
#define PID_KD_POS_R             (0.0)

#define PID_KP_VEL_R             (0.0)
#define PID_KI_VEL_R             (0.0)
#define PID_KD_VEL_R             (0.0)

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

extern struct pid_uint pid_pos_Left;
extern struct pid_uint pid_pos_Right;

extern struct pid_uint pid_vel_Left;
extern struct pid_uint pid_vel_Right;

void PID_Init(void);
int limit_amplitude(int data, int max);
int Position_PID(struct pid_uint *pid);
int Incremental_PID(struct pid_uint *pid);

#endif
