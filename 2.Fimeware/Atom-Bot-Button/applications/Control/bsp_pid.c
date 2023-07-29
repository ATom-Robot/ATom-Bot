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
#include "AT_Math.h"

#include <rtdevice.h>

#define DBG_SECTION_NAME  "PID"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

pid_uint pid_pos[2];
pid_uint pid_vel[2];
pid_uint pid_yaw;

void Loc_level_PID_Init(void)
{
    pid_pos[LEFT].Kp = PID_KP_POS_L;
    pid_pos[LEFT].Ki = PID_KI_POS_L;
    pid_pos[LEFT].Kd = PID_KD_POS_L;

    pid_pos[RIGHT].Kp = PID_KP_POS_R;
    pid_pos[RIGHT].Ki = PID_KI_POS_R;
    pid_pos[RIGHT].Kd = PID_KD_POS_R;

    pid_vel[LEFT].Kp = PID_KP_VEL_L;
    pid_vel[LEFT].Ki = PID_KI_VEL_L;
    pid_vel[LEFT].Kd = PID_KD_VEL_L;

    pid_vel[RIGHT].Kp = PID_KP_VEL_R;
    pid_vel[RIGHT].Ki = PID_KI_VEL_R;
    pid_vel[RIGHT].Kd = PID_KD_VEL_R;

    pid_yaw.Kp = PID_KP_YAW;
    pid_yaw.Ki = PID_KI_YAW;
    pid_yaw.Kd = PID_KD_YAW;
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
int Position_PID(pid_uint *pid, float Target_Value, float Measured_Value)
{
    pid->target = Target_Value;
    pid->reality = Measured_Value;

    /* 计算偏差 */
    pid->Bias = pid->target - pid->reality;
    /* 偏差累积 */
    pid->Integral_bias += pid->Bias;

    pid->Integral_bias = limit_amplitude(pid->Integral_bias, 5000);

    pid->output = (pid->Kp * pid->Bias)
                  + (pid->Ki * pid->Integral_bias)
                  + (pid->Kd * (pid->Bias - pid->Last_bias));

    /* 保存上次偏差 */
    pid->Last_bias = pid->Bias;

    /* 输出结果 */
    return (int)pid->output;
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
int Incremental_PID(pid_uint *pid, float Target_Value, float Measured_Value)
{
    float a = 0.7f;

    pid->target = Target_Value;
    pid->reality = Measured_Value;

    /* 计算偏差 */
    pid->Bias = pid->target - pid->reality;

    pid->low_out = (1 - a) * pid->Bias + a * pid->low_out_last;
    pid->low_out_last = pid->low_out;

    pid->output += (pid->Kp * (pid->low_out - pid->Last_bias))
                   + (pid->Ki * pid->low_out)
                   + (pid->Kd * (pid->low_out - 2 * pid->Last_bias + pid->Prev_bias));

    /* 保存上上次偏差 */
    pid->Prev_bias = pid->Last_bias;
    /* 保存上一次偏差 */
    pid->Last_bias = pid->low_out;

    if (Target_Value > 0)
    {
        if (pid->output < 0) pid->output = 0;
        else if (pid->output > MOTOR_MAX_PULSE) pid->output = MOTOR_MAX_PULSE;
    }
    else if (Target_Value < 0)
    {
        if (pid->output < -MOTOR_MAX_PULSE) pid->output = -MOTOR_MAX_PULSE;
        else if (pid->output > 0) pid->output = 0;
    }

    if (Target_Value == 0) pid->output = 0;

    /* 输出结果 */
    return (int)pid->output;
}

float PID_calculate(float dT_s,           // 周期（单位：秒）
                    float expect,         // 期望值（设定值）
                    float feedback,       // 反馈值
                    pid_uint *pid_arg,
                    _PID_val_st *pid_val,
                    float inte_d_lim,     // 积分误差限幅
                    float inte_lim        // integration limit，积分限幅
                   )
{
    float differential, hz;

    hz = safe_div(1.0f, dT_s, 0);

    pid_val->exp_d = (expect - pid_val->exp_old) * hz;

    differential = (pid_arg->Kd * pid_val->exp_d);

    pid_val->err = (expect - feedback);

    pid_val->err_i += pid_arg->Ki * LIMIT((pid_val->err), -inte_d_lim, inte_d_lim) * dT_s;

    pid_val->err_i = LIMIT(pid_val->err_i, -inte_lim, inte_lim);

    pid_val->out = pid_arg->Kp * pid_val->err + differential + pid_val->err_i;

    pid_val->feedback_old = feedback;
    pid_val->exp_old = expect;

    return (pid_val->out);
}
