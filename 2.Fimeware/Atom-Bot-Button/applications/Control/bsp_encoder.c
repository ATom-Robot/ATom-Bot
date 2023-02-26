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

#include "bsp_encoder.h"

#define DBG_SECTION_NAME  "ENCODER"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    Encoder_Set_Counter(1, ENCODER_MID_VALUE);
    Encoder_Set_Counter(2, ENCODER_MID_VALUE);

    LOG_I("Encoder Init Success\r\n");
}


void Encoder_Set_Counter(int8_t Motor_Num, int16_t count)
{
    switch (Motor_Num)
    {
    case 1:
        __HAL_TIM_SET_COUNTER(&htim3, count);
        break;
    case 2:
        __HAL_TIM_SET_COUNTER(&htim2, count);
        break;
    default:
    {
        LOG_E("Motor_Num ERROR\r\n");
        break;
    }
    }
}

uint16_t Encoder_Get_Counter(int8_t Motor_Num)
{
    uint16_t counter = 0;
    switch (Motor_Num)
    {
    case 1:
        counter = __HAL_TIM_GetCounter(&htim3);
        break;
    case 2:
        counter = __HAL_TIM_GetCounter(&htim2);
        break;
    default:
    {
        LOG_E("Motor_Num ERROR\r\n");
        counter = 0;
        break;
    }
    }
    return counter;
}

uint16_t Encoder_Get_Dir(int8_t Motor_Num)
{
    uint16_t direction = 1;
    switch (Motor_Num)
    {
    case 1:
        direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
        break;
    case 2:
        direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
        break;
    default:
    {
        LOG_E("Motor_Num ERROR\r\n");
        direction = 1;
        break;
    }
    }
    return direction;
}

rt_err_t ReadEncoder_cmd(int argc, const char *argv[])
{
    rt_err_t res = RT_EOK;

    if (argc != 2)
    {
        LOG_E("ERROR Paramter");
        return RT_ERROR;
    }

    int Motor_Num = atoi(argv[1]);

    for (int i = 0; i < 15; i++)
    {
        uint16_t enc = Encoder_Get_Counter(Motor_Num);
        LOG_D("motor[%d] ec count:%d", Motor_Num, enc);
        rt_thread_mdelay(100);
    }

    return res;
}
MSH_CMD_EXPORT(ReadEncoder_cmd, input: num(1: 2) read encoder count);
