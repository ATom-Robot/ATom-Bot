/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66         First version
 */
#include "main.h"
#include <stdlib.h>

#include "bsp_encoder.h"

#define DBG_SECTION_NAME  "ENCODER"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static int g_Encoder_A_Now = 0;
static int g_Encoder_B_Now = 0;

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

static uint16_t Encoder_Get_Counter(uint8_t Motor_Num)
{
    uint16_t counter = 0;

    switch (Motor_Num)
    {
    case ENCODER_ID_A:
        counter = 0x7fff - (short)__HAL_TIM_GetCounter(&htim3);
        __HAL_TIM_SetCounter(&htim3, 0x7fff);
        break;
    case ENCODER_ID_B:
        counter = 0x7fff - (short)__HAL_TIM_GetCounter(&htim2);
        __HAL_TIM_SetCounter(&htim2, 0x7fff);
        break;
    default:
    {
        LOG_E("Motor number error\r\n");
        counter = 0;
        break;
    }
    }
    return counter;
}

int Encoder_Get_Count_Now(uint8_t Motor_Num)
{
    if (Motor_Num == ENCODER_ID_A)
        return g_Encoder_A_Now;

    if (Motor_Num == ENCODER_ID_B)
        return g_Encoder_B_Now;

    return 0;
}

void Encoder_Update_Count(uint8_t Encoder_id)
{
    switch (Encoder_id)
    {
    case ENCODER_ID_A:
    {
        g_Encoder_A_Now -= Encoder_Get_Counter(ENCODER_ID_A);
        break;
    }

    case ENCODER_ID_B:
    {
        g_Encoder_B_Now += Encoder_Get_Counter(ENCODER_ID_B);
        break;
    }

    default:
        break;
    }
}

uint16_t Encoder_Get_Dir(uint8_t Motor_Num)
{
    uint16_t direction = 1;
    switch (Motor_Num)
    {
    case ENCODER_ID_A:
        direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
        break;
    case ENCODER_ID_B:
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

static rt_err_t ReadEncoder_cmd(int argc, const char *argv[])
{
    rt_err_t res = RT_EOK;

    if (argc != 2)
    {
        LOG_E("error paramter");
        return -RT_ERROR;
    }

    int Motor_Num = atoi(argv[1]);

    for (int i = 0; i < 20; i++)
    {
        uint16_t enc = Encoder_Get_Counter(Motor_Num);
        LOG_D("motor[%d] ec count:%d", Motor_Num, enc);
        rt_thread_mdelay(100);
    }

    return res;
}
MSH_CMD_EXPORT(ReadEncoder_cmd, input: num(2/3) read encoder count);
