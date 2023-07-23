/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#include "bsp_pid.h"
#include "bsp_motor.h"

#include <rtdevice.h>

#define DBG_SECTION_NAME  "PID"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

struct pid_uint pid_pos_Left;
struct pid_uint pid_pos_Right;

struct pid_uint pid_vel_Left;
struct pid_uint pid_vel_Right;

void PID_Init(void)
{
    // left wheel pid
    pid_pos_Left.Kp = PID_KP_POS_L;
    pid_pos_Left.Ki = PID_KI_POS_L;
    pid_pos_Left.Kd = PID_KD_POS_L;

	pid_pos_Left.Bias = 0;
	pid_pos_Left.Integral_bias = 0;
	pid_pos_Left.Last_bias = 0;
	pid_pos_Left.Prev_bias = 0;
    pid_pos_Left.reality = 0;
    pid_pos_Left.target = 0;

	pid_vel_Left.Kp = PID_KP_VEL_L;
	pid_vel_Left.Ki = PID_KI_VEL_L;
	pid_vel_Left.Kd = PID_KD_VEL_L;

	pid_vel_Left.Bias = 0;
	pid_vel_Left.Integral_bias = 0;
	pid_vel_Left.Last_bias = 0;
	pid_vel_Left.Prev_bias = 0;
    pid_vel_Left.reality = 0;
    pid_vel_Left.target = 0;

    // right wheel pid
    pid_pos_Right.Kp = PID_KP_POS_R;
    pid_pos_Right.Ki = PID_KI_POS_R;
    pid_pos_Right.Kd = PID_KD_POS_R;

	pid_pos_Right.Bias = 0;
	pid_pos_Right.Integral_bias = 0;
	pid_pos_Right.Last_bias = 0;
	pid_pos_Right.Prev_bias = 0;
    pid_pos_Right.reality = 0;
    pid_pos_Right.target = 0;

	pid_vel_Right.Kp = PID_KP_VEL_R;
	pid_vel_Right.Ki = PID_KI_VEL_R;
	pid_vel_Right.Kd = PID_KD_VEL_R;

	pid_vel_Right.Bias = 0;
	pid_vel_Right.Integral_bias = 0;
	pid_vel_Right.Last_bias = 0;
	pid_vel_Right.Prev_bias = 0;
    pid_vel_Right.reality = 0;
    pid_vel_Right.target = 0;
}

/**************************************************************************
函数功能：限幅
入口参数：电机PWM值
返回  值：限制后的值
**************************************************************************/
int limit_amplitude(int data, int max)
{
    if (data < -max)
        data = -max;
    if (data > max)
        data = max;

    return data;
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：实际位置，目标位置
返回  值：电机PWM
根据位置式离散PID公式
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
pwm代表输出
**************************************************************************/
int Position_PID(struct pid_uint *pid)
{
    int32_t pwm;

    pid->Bias = pid->target - pid->reality;             /* 计算偏差 */
    pid->Integral_bias += pid->Bias;                    /* 偏差累积 */

    pid->Integral_bias = limit_amplitude(pid->Integral_bias, 5000);

    pwm = (pid->Kp * pid->Bias)                         /* 比例环节 */
          + (pid->Ki * pid->Integral_bias)              /* 积分环节 */
          + (pid->Kd * (pid->Bias - pid->Last_bias));   /* 微分环节 */

    pid->Last_bias = pid->Bias;                         /* 保存上次偏差 */

    return pwm;                                         /* 输出结果 */
}

/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
**************************************************************************/
int Incremental_PID(struct pid_uint *pid)
{
    int32_t pwm;

    pid->Bias = pid->target - pid->reality;                         		/* 计算偏差 */

    pwm += (pid->Kp * (pid->Bias - pid->Last_bias))                     	/* 比例环节 */
           + (pid->Ki * pid->Bias)                                  		/* 积分环节 */
           + (pid->Kd * (pid->Bias - 2 * pid->Last_bias + pid->Prev_bias)); /* 微分环节 */

    pid->Prev_bias = pid->Last_bias;    									/* 保存上上次偏差 */
    pid->Last_bias = pid->Bias;     										/* 保存上一次偏差 */

    return pwm;             												/* 输出结果 */
}
