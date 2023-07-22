/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66         First version
 */
#include <stdlib.h>
#include "main.h"
#include "AT_Math.h"

#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"

#define DBG_SECTION_NAME  "MOTOR"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern TIM_HandleTypeDef htim4;

// 编码器10ms前后数据
int leftWheelEncoderNow = 0;
int rightWheelEncoderNow = 0;
int leftWheelEncoderLast = 0;
int rightWheelEncoderLast = 0;

// 计算左右轮速
static double lSpd_mm_s = 0;
static double rSpd_mm_s = 0;

extern int motorLeft_pwm;
extern int motorRight_pwm;

void Motor_Init(void)
{
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    LOG_I("MOTOR Init Success\r\n");
}

static void Motor_m1_pwm(int pwm)
{
	if (pwm == 0)
	{
		PWMA1 = PWMA2 = 0;
		return;
	}

    if (pwm >= 0)
    {
		pwm += Dead_Voltage;
		pwm = limit_amplitude(pwm, MOTOR_MAX_PULSE);

        PWMA1 = 0; PWMA2 = pwm;
    }
    else
    {
		pwm = my_abs(pwm) + Dead_Voltage;
		pwm = limit_amplitude(pwm, MOTOR_MAX_PULSE);

        PWMA1 = pwm; PWMA2 = 0;
    }
}

static void Motor_m2_pwm(int pwm)
{
	if (pwm == 0)
	{
		PWMB1 = PWMB2 = 0;
		return;
	}

    if (pwm >= 0)
    {
		pwm += Dead_Voltage;
		pwm = limit_amplitude(pwm, MOTOR_MAX_PULSE);

        PWMB1 = pwm; PWMB2 = 0;
    }
    else
    {
		pwm = my_abs(pwm) + Dead_Voltage;
		pwm = limit_amplitude(pwm, MOTOR_MAX_PULSE);

        PWMB1 = 0; PWMB2 = pwm;
    }
}

void Motor_Set_Pwm(uint8_t id, int speed)
{
    switch (id)
    {
    case MOTOR_ID_1:
    {
        Motor_m1_pwm(speed);
        break;
    }

    case MOTOR_ID_2:
    {
        Motor_m2_pwm(speed);
        break;
    }

    default:
        break;
    }
}

static rt_err_t SetSpeed_cmd(int argc, const char *argv[])
{
    rt_err_t res = RT_EOK;

    if (argc != 3)
    {
        LOG_E("error paramter");
        return -RT_ERROR;
    }

    int Motor_Num = atoi(argv[1]);
    int Motor_speed = atoi(argv[2]);

    switch (Motor_Num)
    {
    case MOTOR_ID_1:
	{
        Motor_Set_Pwm(Motor_Num, Motor_speed);
		break;
	}
    case MOTOR_ID_2:
	{
        Motor_Set_Pwm(Motor_Num, Motor_speed);
        break;
	}
    default:
        LOG_E("Motor_Num[%d] ERROR\r\n", Motor_Num);
        res = -RT_ERROR;
        break;
    }
    return res;
}
MSH_CMD_EXPORT(SetSpeed_cmd, input: num(1:2) | speed set motor speed)
