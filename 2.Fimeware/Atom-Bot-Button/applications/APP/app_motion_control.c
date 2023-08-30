/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-06-11     Rbb666       the first version
 * 2023-07-29     Rbb666       add angel pid
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "AT_Math.h"
#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "dmp_port_rtt.h"
#include "ano.h"

#define DBG_SECTION_NAME  "app-control"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define HWTIMER_DEV_NAME   "timer4"

typedef struct
{
    int32_t Target_Velocity;
    int32_t Reality_Velocity;

    int32_t Target_Position;
    int32_t Reality_Position;
} velocity_dt;

static velocity_dt wheel_dt[2];
static int32_t motor_pwm[2];

_PID_val_st yaw_val;

static void Motion_Set_PWM(int motor_Left, int motor_Right)
{
    Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
    Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}

int leftSpeedNow = 0;

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    wheel_dt[LEFT].Reality_Velocity = -Encoder_Get_Counter(ENCODER_ID_A);
    wheel_dt[RIGHT].Reality_Velocity = Encoder_Get_Counter(ENCODER_ID_B);

    wheel_dt[LEFT].Reality_Position += wheel_dt[LEFT].Reality_Velocity;
    wheel_dt[RIGHT].Reality_Position += wheel_dt[RIGHT].Reality_Velocity;

//	float left_speed1, left_speed2;
//	left_speed1 = Motor_Speed(wheel_dt[LEFT].Reality_Velocity, 7, 100, 10);

    for (uint8_t i = 0; i < 2; i++)
    {
        Position_PID(&pid_pos[i], wheel_dt[i].Target_Position, wheel_dt[i].Reality_Position);

        motor_pwm[i] = limit_amplitude(pid_pos[i].output, wheel_dt[i].Target_Velocity);

        motor_pwm[i] = Incremental_PID(&pid_vel[i], motor_pwm[i], wheel_dt[i].Reality_Velocity);
    }

    if (angel_control && (ABS(rec_target_yaw - (int)robot_imu_dmp_data.yaw) > 2))
    {
        /* 角度环 */
        PID_calculate(10, rec_target_yaw, robot_imu_dmp_data.yaw, &pid_yaw, &yaw_val, 20, 100);
        //rt_kprintf("%.2f %d\n", yaw_val.err, (int)yaw_val.out);
    }
	else
	{
		angel_control = RT_FALSE;
		yaw_val.out = 0;
	}

    Motion_Set_PWM(motor_pwm[LEFT] + yaw_val.out, motor_pwm[RIGHT] - yaw_val.out);

    return RT_EOK;
}

static void Motion_Control_20ms(void *parameter)
{
    while (1)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            wheel_dt[i].Target_Velocity = Rpm_Encoder_Cnt(rec_target_rpm, 7, 100, 10);
            wheel_dt[i].Target_Position = Num_Encoder_Cnt(rec_target_motor_num, 7, 100);
        }

        ano_send_user_data(1, wheel_dt[LEFT].Target_Position,   \
                           wheel_dt[LEFT].Reality_Position,     \
                           wheel_dt[LEFT].Target_Velocity,      \
                           wheel_dt[LEFT].Reality_Velocity);

        rt_thread_mdelay(10);
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

int control_hwtimer(void)
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
    timeout_s.usec = 10000;
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    return ret;
}
