/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-20     cheung       first version
 */

#ifndef __DRV_I2C__
#define __DRV_I2C__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>

#ifdef BSP_USING_HW_I2C

#define BSP_I2C2_CLOCK	400000

/* stm32 config class */
typedef void (*pI2CInit)(rt_uint32_t speed);
struct stm32_hard_i2c_config
{
    const char *bus_name;
    const char *scl_pin_name;
    const char *sda_pin_name;
    rt_uint32_t speed;
    pI2CInit   pInitFunc;
    I2C_HandleTypeDef *pHi2c;
    struct rt_i2c_bus_device i2c_bus;

    /* notice and lock define */
    struct rt_completion tx_notice;
    struct rt_completion rx_notice;
};

int rt_hw_hardi2c_init(void);

#endif

#endif /* RT_USING_I2C */
