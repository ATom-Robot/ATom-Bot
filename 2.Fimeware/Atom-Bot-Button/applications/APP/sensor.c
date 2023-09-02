#include "BSP_Joint.h"
#include "dmp_port_rtt.h"
#include "vl53l0x.h"
#include "drv_sensor.h"
#include "ano.h"

#include <rtdevice.h>

#define DBG_SECTION_NAME  "sensor"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

extern int target_angle;
extern struct Joint_device joint[JOINT_SIZE];

void sensor_entry()
{
    while (1)
    {
        UpdateJointAngle_2(&joint[1], target_angle);
        UpdateJointAngle_2(&joint[2], -target_angle);

        MPU6050_DMP_GetData(&robot_imu_dmp_data);

        distence_sensor_get();

        ano_send_user_data(1, (int)joint[1].config.angle,   \
                           (int)joint[2].config.angle,      \
                           0,                           	\
                           0);

        rt_thread_mdelay(10);
    }
}

int sensor_thread_create(void)
{
    rt_thread_t sensor_thread;

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

    return RT_EOK;
}
