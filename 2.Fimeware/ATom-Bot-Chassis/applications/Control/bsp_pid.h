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
#define PID_KP_POS_L            (0.02f)
#define PID_KI_POS_L            (0.0f)
#define PID_KD_POS_L            (0.050f)

#define PID_KP_VEL_L            (0.900f)
#define PID_KI_VEL_L            (0.010f)
#define PID_KD_VEL_L            (0.0f)

// right wheel pid
#define PID_KP_POS_R            (0.02f)
#define PID_KI_POS_R            (0.0f)
#define PID_KD_POS_R            (0.050f)

#define PID_KP_VEL_R            (0.900f)
#define PID_KI_VEL_R            (0.010f)
#define PID_KD_VEL_R            (0.0f)

// yaw pid
#define PID_KP_YAW              (0.050f)
#define PID_KI_YAW              (0.000f)
#define PID_KD_YAW              (0.00f)

enum
{
    LEFT = 0,
    RIGHT = 1
};

typedef struct
{
    float Kp;               //比例
    float Ki;               //积分
    float Kd;               //微分

    float Bias;
    float Integral_bias;
    float Last_bias;        //调节量
    float Prev_bias;        //调节量
    float output;           //输出

    float low_out;
    float low_out_last;

    int32_t target;         //速度设置
    int32_t reality;        //当前速度
} pid_uint;

typedef struct
{
    float err;
    float exp_old;
    float feedback_old;

    float fb_d_ex;
    float exp_d;
    float err_i;

    float out;
} _PID_val_st;

extern pid_uint pid_pos[2];
extern pid_uint pid_vel[2];
extern pid_uint pid_yaw;

void Loc_level_PID_Init(void);
int limit_amplitude(int data, int max);
int Position_PID(pid_uint *pid, float Target_Value, float Measured_Value);
int Incremental_PID(pid_uint *pid, float Target_Value, float Measured_Value);
int PID_calculate(float dT_s, float expect, float feedback, pid_uint *pid_arg, _PID_val_st *pid_val, float inte_d_lim, float inte_lim);

#endif
