
/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-vl53l0x>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-16     panrui      the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"
#include "vl53l0x.h"

static void read_distance_entry(void *parameter)
{
    rt_device_t temp_dev = RT_NULL;
    struct rt_sensor_data temp_data;
    rt_size_t res = 0;
	rt_uint32_t index = 0;
	
	temp_dev = rt_device_find("tof_vl53l0x");
    if (temp_dev == RT_NULL)
    {
	  	 rt_kprintf("not found tof_vl53l0x device\r\n");
        return;
    }

    if (rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open tof_vl53l0x failed\r\n");
        return;
    }
	
    while (1)
    {
		res = rt_device_read(temp_dev, 0, &temp_data, 1);
        if (res == 0)
        {
            rt_kprintf("read data failed! result is %d\n", res);;
			rt_device_close(temp_dev);
            return;
        }
        else
        {
        	rt_kprintf("distance[%dmm],timestamp[%d]\r\n",temp_data.data.proximity,
					   temp_data.timestamp);
        }
		
		if (index++ >= 5)
		{
			rt_device_close(temp_dev);
			break;
		}
        rt_thread_delay(100);
    }
}

static int read_distance_sample(void)
{
    rt_thread_t distance_thread;

    distance_thread = rt_thread_create("tof_r",
                                     read_distance_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (distance_thread != RT_NULL)
    {
        rt_thread_startup(distance_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(read_distance_sample);

static int rt_hw_vl53l0x_port(void)
{
    struct rt_sensor_config cfg;
    	
	cfg.intf.dev_name = "i2c2"; 		/* i2c bus */
    cfg.intf.user_data = (void *)0x29;	/* i2c slave addr */
    rt_hw_vl53l0x_init("vl53l0x", &cfg, 57);/* xshutdown ctrl pin */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_vl53l0x_port);
