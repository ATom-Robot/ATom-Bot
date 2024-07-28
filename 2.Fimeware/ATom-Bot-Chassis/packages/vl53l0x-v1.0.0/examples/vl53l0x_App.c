
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
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define INT_PIN GET_PIN(A, 2)

static rt_int32_t *dis_sensor_data;
static rt_device_t vl5310x_dev = RT_NULL;
static struct rt_sensor_data temp_data;
static struct rt_completion vl53l0_sem;

static void distance_sensor_probe(void)
{
    vl5310x_dev = rt_device_find("tof_vl53l0x");
    if (vl5310x_dev == RT_NULL)
    {
        rt_kprintf("not found tof_vl53l0x device\n");
        return;
    }

    if (rt_device_open(vl5310x_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open tof_vl53l0x failed\n");
        return;
    }
}

#if VL53L0X_USING_INT
static void vl_int_callback(void *args)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_completion_done(&vl53l0_sem);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

static int get_distence_sensor_data(void)
{
    rt_kprintf("Distance (mm): %d\n", *dis_sensor_data);

    return 0;
}
MSH_CMD_EXPORT(get_distence_sensor_data, Get distence sensor data)

rt_int32_t distence_sensor_get(void)
{
#if VL53L0X_USING_INT
    VL53L0X_RangingMeasurementData_t measure;
    VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &measure);
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);

    dis_sensor_data = &measure.RangeMilliMeter;
#endif
    rt_size_t res = 0;
    res = rt_device_read(vl5310x_dev, 0, &temp_data, 1);
    if (res == 0)
    {
        rt_kprintf("read data failed! result is %d\n", res);
		return 0;
    }
    dis_sensor_data = &temp_data.data.proximity;

    return *dis_sensor_data;
}

int vl53l0x_port(void)
{
    struct rt_sensor_config cfg;
    static rt_err_t init_flag;

    cfg.intf.dev_name = "i2c1";         /* i2c bus */
    cfg.intf.user_data = (void *)0x29;  /* i2c slave addr */
    init_flag = rt_hw_vl53l0x_init("vl53l0x", &cfg, RT_NULL);/* xshutdown ctrl pin */
    if (init_flag != RT_EOK)
    {
        rt_kprintf("Distence sensor init fail!\n");
        return -RT_ERROR;
    }
    rt_kprintf("Distence sensor init success\n");

#if VL53L0X_USING_INT
    rt_pin_mode(INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(INT_PIN, PIN_IRQ_MODE_FALLING, vl_int_callback, RT_NULL);
    rt_pin_irq_enable(INT_PIN, PIN_IRQ_ENABLE);

    rt_completion_init(&vl53l0_sem);
#endif
	
	distance_sensor_probe();

    return RT_EOK;
}
