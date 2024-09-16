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
#define __ANO_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "command.h"

// 开启上位机调参模式
#define USING_ANO_DEBUG		0

extern int rec_target_yaw;
extern int rec_target_rpm[2];
extern float rec_target_motor_num;
extern rt_bool_t angel_control;

int ano_init(void *param);
int ano_set_device(const char *device_name);
int ano_send_power(uint16_t votage, uint16_t current);
int ano_send_user_data(uint8_t number, int16_t d0, int16_t d1, int16_t d2, int16_t d3, int16_t d4, int16_t d5, int16_t d6);
int ano_send_motorpwm(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7, uint16_t m_8);

command_sender_t ano_get_sender(void);

#endif