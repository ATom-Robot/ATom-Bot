/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66			First version
 */
#include "main.h"
#include <stdlib.h>

#include "bsp_motor.h"

#define DBG_SECTION_NAME  "MOTOR"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern TIM_HandleTypeDef htim4;

void Motor_Init(void)
{
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);

    LOG_I("MOTOR Init Success\r\n");
}

static void MOTOR1_SetSpeed(int16_t speed)
{
    uint16_t temp;

	if (temp > 1000) temp = 1000;

    if (speed > 0)
    {
        temp = speed;
        PWMA1 = 0;
		PWMA2 = temp;
    }
    else if (speed < 0)
    {
        temp = (-speed);
        PWMA1 = temp;
		PWMA2 = 0;
    }
    else
    {
        temp = 0;
		PWMA1 = PWMA2 = temp;
    }
}

static void MOTOR2_SetSpeed(int16_t speed)
{
    uint16_t temp;

	if (temp > 1000) temp = 1000;

    if (speed > 0)
    {
        temp = speed;
		PWMB1 = 0;
		PWMB2 = temp;
    }
    else if (speed < 0)
    {
        temp = (-speed);
        PWMB1 = temp;
		PWMB2 = 0;
    }
    else
    {
        temp = 0;
		PWMB1 = PWMB2 = temp;
    }
}

void MOTOR_SetSpeed(int8_t Motor_Num, int16_t speed)
{
    switch (Motor_Num)
    {
    case 1:
        MOTOR1_SetSpeed(speed);
        break;
    case 2:
        MOTOR2_SetSpeed(speed);
        break;
    default:
        LOG_E("Motor_Num[%d] ERROR\r\n", Motor_Num);
        break;
    }
}

rt_err_t SetSpeed_cmd(int argc, const char*argv[])
{
	rt_err_t res = RT_EOK;

	if (argc != 3)
	{
		LOG_E("ERROR Paramter");
		return RT_ERROR;
	}
	
	int Motor_Num = atoi(argv[1]);
	int Motor_speed = atoi(argv[2]);

    switch (Motor_Num)
    {
    case 1:
        MOTOR1_SetSpeed(Motor_speed);
        break;
    case 2:
        MOTOR2_SetSpeed(Motor_speed);
        break;
    default:
        LOG_E("Motor_Num[%d] ERROR\r\n", Motor_Num);
		res = RT_ERROR;
        break;
    }
	return res;
}
MSH_CMD_EXPORT(SetSpeed_cmd, input:num(1:2)|speed set motor speed);
