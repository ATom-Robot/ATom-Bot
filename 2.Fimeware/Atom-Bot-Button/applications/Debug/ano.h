/*
 * Copyright (c) 2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-08-26     sogwms       The first version
 */

#ifndef __ANO_H__
#define	__ANO_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "command.h"

extern int rec_target_rpm;
extern float rec_target_motor_num;

int ano_init(void *param);
int ano_set_device(const char *device_name);
int ano_send_power(uint16_t votage, uint16_t current);
int ano_send_user_data(uint8_t number, int32_t d0, int32_t d1, int32_t d2, int32_t d3);
int ano_send_motorpwm(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7, uint16_t m_8);

command_sender_t ano_get_sender(void);

#endif
