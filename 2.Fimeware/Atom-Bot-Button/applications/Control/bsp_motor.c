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

void Motor_Init(void)
{
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);

    LOG_I("MOTOR Init Success\r\n");
}

static void Motor_m1_pwm(int speed)
{
    if (speed >= 0)
    {
        PWMA1 = 0;
        PWMA2 = speed;
    }
    else
    {
        PWMA1 = my_abs(speed);
        PWMA2 = 0;
    }
}

static void Motor_m2_pwm(int speed)
{
    if (speed >= 0)
    {
        PWMB1 = speed;
        PWMB2 = 0;
    }
    else
    {
        PWMB1 = 0;
        PWMB2 = my_abs(speed);
    }
}

void Motor_Set_Pwm(uint8_t id, int speed)
{
    // limit
    if (speed > MOTOR_MAX_PULSE) speed = MOTOR_MAX_PULSE;
    if (speed < -MOTOR_MAX_PULSE) speed = -MOTOR_MAX_PULSE;

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
    case MOTOR_ID_2:
        Motor_Set_Pwm(Motor_Num, Motor_speed);
        break;
    default:
        LOG_E("Motor_Num[%d] ERROR\r\n", Motor_Num);
        res = -RT_ERROR;
        break;
    }
    return res;
}
MSH_CMD_EXPORT(SetSpeed_cmd, input: num(1: 2) | speed set motor speed)

void Get_Motor_Speed(int *leftSpeed, int *rightSpeed)
{
    Encoder_Update_Count(ENCODER_ID_A);
    leftWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_A);

    Encoder_Update_Count(ENCODER_ID_B);
    rightWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_B);

    *leftSpeed = (leftWheelEncoderNow - leftWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;
    *rightSpeed = (rightWheelEncoderNow - rightWheelEncoderLast) * WHEEL_CIRCUMFERENCE_CM;

    lSpd_mm_s = (double)(leftWheelEncoderNow - leftWheelEncoderLast) * ENCODER_CNT_10MS_2_SPD_MM_S;
    rSpd_mm_s = (double)(rightWheelEncoderNow - rightWheelEncoderLast) * ENCODER_CNT_10MS_2_SPD_MM_S;

    // record
    leftWheelEncoderLast = leftWheelEncoderNow;
    rightWheelEncoderLast = rightWheelEncoderNow;
}
