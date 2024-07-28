/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-03-05     Rbb66         First version
 */

#ifndef DRV_SENSOR_H
#define DRV_SENSOR_H

int sensor_thread_create(void);
void sensor_acquire(void);
void sensor_release(void);
#endif
