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

#define DBG_SECTION_NAME  "PID"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

struct pid_uint pid_wheel_Left;
struct pid_uint pid_wheel_Right;

void reset_PID(struct pid_uint *p)
{
    p->U_kk = 0;
    p->ekk = 0;
    p->ekkk = 0;
    p->Adjust = 0;
    p->speedNow = 0;
    p->speedSet = 0;
}

void reset_Uk(struct pid_uint *p)
{
    p->U_kk = 0;
    p->ekk = 0;
    p->ekkk = 0;
}

void PID_Init(void)
{
    // left wheel pid
    pid_wheel_Left.Kp = 1024 * PID_KP_DEF_L;
    pid_wheel_Left.Ki = 1024 * PID_KI_DEF_L;
    pid_wheel_Left.Kd = 1024 * PID_KD_DEF_L;
    pid_wheel_Left.Ur = 1024 * MOTOR_MAX_PULSE * 2;
    pid_wheel_Left.Adjust = 0;
    pid_wheel_Left.En = 1;
    pid_wheel_Left.speedSet = 0;
    pid_wheel_Left.speedNow = 0;
    reset_Uk(&pid_wheel_Left);

    // right wheel pid
    pid_wheel_Right.Kp = 1024 * PID_KP_DEF_R;
    pid_wheel_Right.Ki = 1024 * PID_KI_DEF_R;
    pid_wheel_Right.Kd = 1024 * PID_KD_DEF_R;
    pid_wheel_Right.Ur = 1024 * MOTOR_MAX_PULSE * 2;
    pid_wheel_Right.Adjust = 0;
    pid_wheel_Right.En = 1;
    pid_wheel_Right.speedSet = 0;
    pid_wheel_Right.speedNow = 0;
    reset_Uk(&pid_wheel_Right);
}

int32_t PID_common(int set, int jiance, struct pid_uint *p)
{
    int ek = 0, U_k = 0;

    ek = jiance - set;

    U_k = p->U_kk + p->Kp * (ek - p->ekk) + p->Ki * ek + p->Kd * (ek - 2 * p->ekk + p->ekkk);

    p->U_kk = U_k;
    p->ekkk = p->ekk;
    p->ekk = ek;

    if (U_k > (p->Ur))
        U_k = p->Ur;
    if (U_k < -(p->Ur))
        U_k = -(p->Ur);

    return U_k >> 10;
}

void Pid_Which(struct pid_uint *pl, struct pid_uint *pr, float yaw)
{
    // left wheel speed pid
    if (pl->En == 1)
    {
        pl->Adjust = -PID_common(pl->speedSet, pl->speedNow, pl);
    }
    else
    {
        pl->Adjust = 0;
        reset_Uk(pl);
        pl->En = 2;
    }

    // right wheel speed pid
    if (pr->En == 1)
    {
        pr->Adjust = -PID_common(pr->speedSet, pr->speedNow, pr);
    }
    else
    {
        pr->Adjust = 0;
        reset_Uk(pr);
        pr->En = 2;
    }
}

// pid ctrl
void Pid_Ctrl(int *leftMotor, int *rightMotor, float yaw)
{
    int temp_l = *leftMotor;
    int temp_r = *rightMotor;

    Pid_Which(&pid_wheel_Left, &pid_wheel_Right, yaw);

    temp_l += pid_wheel_Left.Adjust;
    temp_r += pid_wheel_Right.Adjust;

    if (temp_l > MOTOR_MAX_PULSE)  temp_l = MOTOR_MAX_PULSE;
    if (temp_l < -MOTOR_MAX_PULSE) temp_l = -MOTOR_MAX_PULSE;
    if (temp_r > MOTOR_MAX_PULSE)  temp_r = MOTOR_MAX_PULSE;
    if (temp_r < -MOTOR_MAX_PULSE) temp_r = -MOTOR_MAX_PULSE;

    *leftMotor  = temp_l;
    *rightMotor = temp_r;
}
