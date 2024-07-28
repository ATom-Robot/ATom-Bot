/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-8-6       sogwms       the first version
 */

#include <stdint.h>

#define AK8963_I2C_ADDR                 ((uint8_t)0x0C)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_REG_DeviceID             ((uint8_t)0x48)

/* Read-only Reg */
#define AK8963_REG_WIA                  ((uint8_t)0x00)
#define AK8963_REG_INFO                 ((uint8_t)0x01)
#define AK8963_REG_ST1                  ((uint8_t)0x02)
#define AK8963_REG_HXL                  ((uint8_t)0x03)
#define AK8963_REG_HXH                  ((uint8_t)0x04)
#define AK8963_REG_HYL                  ((uint8_t)0x05)
#define AK8963_REG_HYH                  ((uint8_t)0x06)
#define AK8963_REG_HZL                  ((uint8_t)0x07)
#define AK8963_REG_HZH                  ((uint8_t)0x08)
#define AK8963_REG_ST2                  ((uint8_t)0x09)

/* Write/Read Reg */
#define AK8963_REG_CNTL1                ((uint8_t)0x0A)
#define AK8963_REG_CNTL2                ((uint8_t)0x0B)
#define AK8963_REG_ASTC                 ((uint8_t)0x0C)
#define AK8963_REG_TS1                  ((uint8_t)0x0D)
#define AK8963_REG_TS2                  ((uint8_t)0x0E)
#define AK8963_REG_I2CDIS               ((uint8_t)0x0F)

/* Read-only Reg (ROM) */
#define AK8963_REG_ASAX                 ((uint8_t)0x10)
#define AK8963_REG_ASAY                 ((uint8_t)0x11)
#define AK8963_REG_ASAZ                 ((uint8_t)0x12)

/* Status */
#define AK8963_STATUS_DRDY              ((uint8_t)0x01)
#define AK8963_STATUS_DOR               ((uint8_t)0x02)
#define AK8963_STATUS_HOFL              ((uint8_t)0x08)
