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

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    Encoder_Set_Counter(ENCODER_ID_A, 0);
    Encoder_Set_Counter(ENCODER_ID_B, 0);

    LOG_I("Encoder Init Success\r\n");
}

void Encoder_Set_Counter(int8_t Motor_Num, int16_t count)
{
    switch (Motor_Num)
    {
    case ENCODER_ID_A:
        TIM3->CNT = count;
        break;
    case ENCODER_ID_B:
        TIM2->CNT = count;
        break;
    default:
    {
        LOG_E("Motor_Num ERROR\r\n");
        break;
    }
    }
}

int32_t Encoder_Get_Counter(uint8_t Motor_Num)
{
    int32_t counter = 0;

    switch (Motor_Num)
    {
    case ENCODER_ID_A:
        counter = (short)TIM3->CNT;
        TIM3->CNT = 0;
        break;
    case ENCODER_ID_B:
        counter = (short)TIM2->CNT;
        TIM2->CNT = 0;
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

/**************************************************************************
功    能: 计算实际转速
输    入: encoder_cnt：脉冲数；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返回  值: 车轮转速 rpm
**************************************************************************/
float Motor_Speed(int encoder_cnt, uint16_t ppr, uint16_t ratio, uint16_t cnt_time)
{
    encoder_cnt = abs(encoder_cnt);
    return (encoder_cnt / 4 / ppr / ratio) * (1000 / cnt_time) * 60; /* 4倍频 */
}

/**************************************************************************
功    能: 计算转数对应编码器脉冲数
输    入: num：转数；ppr：码盘数；ratio：减速比
返 回 值: 电机脉冲数
**************************************************************************/
long Num_Encoder_Cnt(float num, uint16_t ppr, uint16_t ratio)
{
    return (num * ratio * ppr * 4); /* 4倍频 */
}

/**************************************************************************
功    能: 计算转速对应编码器脉冲数
输    入: rpm：转速；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返 回 值: 电机脉冲数
**************************************************************************/
long Rpm_Encoder_Cnt(float rpm, uint16_t ppr, uint16_t ratio, uint16_t cnt_time)
{
    return (rpm * ratio * ppr * 4) / (60 * 1000 / cnt_time); /* 4倍频 */
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

    for (int i = 0; i < 50; i++)
    {
        int enc = Encoder_Get_Counter(Motor_Num);
        LOG_D("motor[%d] ec count:%d", Motor_Num, enc);
        rt_thread_mdelay(100);
    }

    return res;
}
MSH_CMD_EXPORT(ReadEncoder_cmd, input: num(2 / 3) read encoder count);
