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
#define PID_KP_DEF_L             (0.1)
#define PID_KI_DEF_L             (0.0)
#define PID_KD_DEF_L             (4.0)

// right wheel pid
#define PID_KP_DEF_R             (0.1)
#define PID_KI_DEF_R             (0.0)
#define PID_KD_DEF_R             (4.0)

struct pid_uint
{
    int32_t U_kk;		//上一次的输出量
    int32_t ekk;		//上一次的输入偏差
    int32_t ekkk;		//前一次的输入偏差
    int32_t Ur;			//限幅输出值,需初始化
    int32_t Kp;			//比例
    int32_t Ki;			//积分
    int32_t Kd;			//微分

    uint8_t  En;		//开关
    int16_t Adjust;		//调节量
    int16_t speedSet;	//速度设置
    int16_t speedNow;	//当前速度
};

#endif
