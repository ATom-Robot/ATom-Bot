/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-03-05     Rbb66         First version
 */

#include "bsp_vin.h"

#define DBG_SECTION_NAME  "VIN_TASK"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

int battery_status = IS_FULL;

#define VOLTAGE_DROP		0.13214f
#define VOLTAGE_CHG			3.85f
#define VOLTAGE_OFF			3.72f

float battery_voltage;

//100ms
void vin_task(void *argument)
{
	MX_ADC1_Init();
    VIN_Init();

    while (1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;

        if (battery_voltage < VOLTAGE_CHG)
        {
            //需要充电
            battery_status = NEED_CHARGE;

            if (battery_voltage < VOLTAGE_OFF)
            {
                rt_thread_mdelay(5000);
                battery_voltage = get_battery_voltage() + VOLTAGE_DROP;

                //电压持续小于VOLTAGE_OFF 5秒，进入停机保护状态
                if (battery_voltage < VOLTAGE_OFF)
                {
                    battery_status = NEED_POWEROFF;
                    //停止电机放在电机的任务里

                    //停止检测
                    LOG_I("电压过低，停止工作\r\n");
                    while (1)
                    {
                        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
                        if (battery_voltage >= VOLTAGE_OFF)
                        {
                            LOG_I("电压达到要求，重新工作\r\n");
                            break;
                        }
                        rt_thread_mdelay(200);
                    }
                }
            }
        }
        else
        {
            //电池正常
            battery_status = IS_FULL;
        }

        rt_thread_mdelay(200);
    }
}

int ADC_Battery_Detection()
{
    rt_thread_t tid;

    tid = rt_thread_create("voltage", vin_task, RT_NULL,
                           1024, 20, 15);
    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);
	
	return RT_EOK;
}
INIT_APP_EXPORT(ADC_Battery_Detection);

int Check_Voltage_Data(void)
{
	battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
	LOG_I("Battery currnet voltage:%f", battery_voltage );
	return RT_EOK;
}
MSH_CMD_EXPORT(Check_Voltage_Data, To Check robot battery voltage);
