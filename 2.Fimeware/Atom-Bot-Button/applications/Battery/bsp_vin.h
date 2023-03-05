/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-03-05     Rbb66         First version
 */

#ifndef BSP_VIN_H
#define BSP_VIN_H

#include <rtthread.h>
#include <board.h>

enum
{
    IS_FULL = 1,
    NEED_CHARGE,
    NEED_POWEROFF
};

void VIN_Init(void);

float get_temprate(void);
void init_vrefint_reciprocal(void);
float get_battery_voltage(void);

uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
float adcx_get_chx_value_average(ADC_HandleTypeDef *ADCx, uint32_t ch, uint8_t times);


#endif
