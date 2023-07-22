
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

#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "ano.h"

#define DBG_SECTION_NAME  "app-control"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define HWTIMER_DEV_NAME   "timer4"

#define MAX_STOP_COUNT 100

int motorLeft_pwm = 0;
int motorRight_pwm = 0;

struct velocity_dt
{
    int Target_Velocity;
    int Reality_Velocity;

    int Target_Position;
    int Reality_Position;
};
struct velocity_dt left_velocity;
struct velocity_dt right_velocity;

static void Motion_Set_PWM(int motor_Left, int motor_Right)
{
    Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
    Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    if (left_velocity.Target_Velocity || right_velocity.Target_Velocity)
    {
        left_velocity.Reality_Velocity = Encoder_Get_Counter(ENCODER_ID_A);
        right_velocity.Reality_Velocity = Encoder_Get_Counter(ENCODER_ID_B);

        left_velocity.Target_Position += left_velocity.Reality_Velocity;
        right_velocity.Target_Position += right_velocity.Reality_Velocity;

    }

    return 0;
}

int hwtimer_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;
    rt_device_t hw_dev = RT_NULL;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq = 10000;

    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    rt_device_set_rx_indicate(hw_dev, timeout_cb);
    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);

    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    timeout_s.sec = 0;
    timeout_s.usec = 20000;
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    return ret;
}

static void Motion_Control_20ms(void *parameter)
{
    while (1)
    {
        left_velocity.Target_Position = Num_Encoder_Cnt(rec_target_motor_num, 7, 100);

        rt_thread_mdelay(20);
    }
}

void app_motion_ctrl_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("ctrl", Motion_Control_20ms, RT_NULL,
                           1024, 15, 20);
    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);
}

static int set_wheel_speed(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter");
        return RT_ERROR;
    }

    int left = atoi(argv[1]);
    int right = atoi(argv[2]);
    LOG_I("set left speed:%d,right speed:%d\n", left, right);

    return 0;
}
MSH_CMD_EXPORT(set_wheel_speed, set wheel speed)
