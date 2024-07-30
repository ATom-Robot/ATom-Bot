/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

#define RT_APP_PART_ADDR 0x0800C000

extern const struct fal_flash_dev stm32_onchip_flash;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &stm32_onchip_flash,                                             \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE                                                                      \
{                                                                                           \
    {FAL_PART_MAGIC_WORD,        "bl",      "onchip_flash",     0,          48*1024, 0},    \
    {FAL_PART_MAGIC_WORD,       "app",      "onchip_flash",     48*1024,    110*1024, 0},   \
    {FAL_PART_MAGIC_WORD,   "download",     "onchip_flash",     158*1024,   94*1024,  0},   \
}
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
