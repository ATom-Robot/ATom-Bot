
/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-06-11     Rbb666       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "sensor.h"
#include "vl53l0x.h"
#include "vl53l0x_api.h"

#define DBG_SECTION_NAME  "vl53l0"
#define DBG_LEVEL         DBG_ERROR
#include <rtdbg.h>

#define INT_PIN GET_PIN(B, 1)

static rt_sem_t vl53l0_sem = RT_NULL;

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
        rt_sem_take(vl53l0_sem, RT_WAITING_FOREVER);

        VL53L0X_RangingMeasurementData_t measure;
        VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &measure);
        if (measure.RangeStatus != 4)   // phase failures have incorrect data
        {
            LOG_D("Distance (mm): %d\n", measure.RangeMilliMeter);
        }
        VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    }
}

static int read_distance_sample(void)
{
    rt_thread_t distance_thread;

    vl53l0_sem = rt_sem_create("vlx", 0, RT_IPC_FLAG_PRIO);
    if (vl53l0_sem == RT_NULL)
    {
        LOG_E("create dynamic semaphore failed.\n");
        return -1;
    }

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

static void vl_int_callback(void *args)
{
    rt_sem_release(vl53l0_sem);
}

static int rt_hw_vl53l0x_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "i2c1";         /* i2c bus */
    cfg.intf.user_data = (void *)0x29;  /* i2c slave addr */
    rt_hw_vl53l0x_init("vl53l0x", &cfg, RT_NULL);/* xshutdown ctrl pin */

    rt_pin_mode(INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(INT_PIN, PIN_IRQ_MODE_FALLING, vl_int_callback, RT_NULL);
    rt_pin_irq_enable(INT_PIN, PIN_IRQ_ENABLE);

	read_distance_sample();

    return RT_EOK;
}
//INIT_DEVICE_EXPORT(rt_hw_vl53l0x_port);
MSH_CMD_EXPORT(rt_hw_vl53l0x_port, VL5310 init)