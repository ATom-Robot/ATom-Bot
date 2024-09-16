#include "BSP_Joint.h"
#include "bsp_vin.h"
#include "vl53l0x.h"
#include "dmp_port_rtt.h"
#include "drv_sensor.h"
#include "ano.h"

#include <rtdevice.h>

#define DBG_SECTION_NAME  "sensor"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static rt_mutex_t i2c_mutex = RT_NULL;
static rt_thread_t sensor_thread;

extern struct Joint_device joint[JOINT_SIZE];
static int distance = 0;

static void move_backward(void);

void sensor_entry()
{
    while (1)
    {
        if (RT_EOK == rt_mutex_take(i2c_mutex, RT_WAITING_FOREVER))
        {
            UpdateServoAngle_1(&joint[1]);
            UpdateServoAngle_1(&joint[2]);

            rt_mutex_release(i2c_mutex);
        }

        MPU6050_DMP_GetData(&robot_imu_dmp_data);

		/* 跌落检测 */
		// move_backward();

        ano_send_user_data(5, (int)joint[1].config.angle,   \
                           (int)joint[2].config.angle,      \
                           (int)robot_imu_dmp_data.pitch,   \
                           (int)robot_imu_dmp_data.roll,    \
                           (int)robot_imu_dmp_data.yaw,     \
                           (int)(get_battery_data() * 100));

        rt_thread_mdelay(50);
    }
}

/* 自动检测倒车函数 */
static void move_backward(void)
{
	distance = distence_sensor_get();

	if (distance >= 25)
	{
		rec_target_motor_num = -2;
		rec_target_rpm[0] = rec_target_rpm[1] = 120;
	}
}

void sensor_acquire(void)
{
    rt_thread_t task = rt_thread_self();
    if (sensor_thread != task)
    {
        rt_mutex_take(i2c_mutex, RT_WAITING_FOREVER);
    }
}

void sensor_release(void)
{
    rt_thread_t task = rt_thread_self();
    if (sensor_thread != task)
    {
        rt_mutex_release(i2c_mutex);
    }
}

int sensor_thread_create(void)
{
    sensor_thread = rt_thread_create("sensor",
                                     sensor_entry,
                                     RT_NULL,
                                     1024,
                                     25,
                                     20);
    if (sensor_thread != RT_NULL)
    {
        rt_thread_startup(sensor_thread);
    }

    i2c_mutex = rt_mutex_create("j_mutex", RT_IPC_FLAG_PRIO);
    if (i2c_mutex == RT_NULL)
    {
        rt_kprintf("create dynamic mutex failed.\n");
        return -1;
    }

    return RT_EOK;
}
