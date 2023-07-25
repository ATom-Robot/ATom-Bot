
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

#include "AT_Math.h"
#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "ano.h"

#define DBG_SECTION_NAME  "app-control"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define HWTIMER_DEV_NAME   "timer4"

static int32_t motorLeft_pwm = 0;
static int32_t motorRight_pwm = 0;

struct velocity_dt
{
    int32_t Target_Velocity;
    int32_t Reality_Velocity;

    int32_t Target_Position;
    int32_t Reality_Position;
};
static struct velocity_dt left_wheel;
static struct velocity_dt right_wheel;

static void Motion_Set_PWM(int motor_Left, int motor_Right)
{
    Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
    Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    left_wheel.Reality_Velocity = Encoder_Get_Counter(ENCODER_ID_A);
    right_wheel.Reality_Velocity = Encoder_Get_Counter(ENCODER_ID_B);

    left_wheel.Reality_Position += left_wheel.Reality_Velocity;
    right_wheel.Reality_Position += right_wheel.Reality_Velocity;

	/* 位置PID控制器 */
	motorLeft_pwm = Position_PID(&pid_pos_Left, left_wheel.Target_Position, left_wheel.Reality_Position);
	motorRight_pwm = Position_PID(&pid_pos_Right, right_wheel.Target_Position, right_wheel.Reality_Position);

	motorLeft_pwm = limit_amplitude(motorLeft_pwm, left_wheel.Target_Velocity);
	motorRight_pwm = limit_amplitude(motorRight_pwm, right_wheel.Target_Velocity);

	/* 增量PID控制器 */
	motorLeft_pwm = Incremental_PID(&pid_vel_Left, motorLeft_pwm, left_wheel.Reality_Velocity);
	motorRight_pwm = Incremental_PID(&pid_vel_Right, motorRight_pwm, right_wheel.Reality_Velocity);

	Motion_Set_PWM(motorLeft_pwm, motorRight_pwm);

    return RT_EOK;
}

static void Motion_Control_20ms(void *parameter)
{
    while (1)
    {
		left_wheel.Target_Velocity = Rpm_Encoder_Cnt(rec_target_rpm, 7, 100, 10);
		right_wheel.Target_Velocity = Rpm_Encoder_Cnt(rec_target_rpm, 7, 100, 10);

        left_wheel.Target_Position = Num_Encoder_Cnt(rec_target_motor_num, 7, 100);
        right_wheel.Target_Position = Num_Encoder_Cnt(rec_target_motor_num, 7, 100);

//        ano_send_user_data(1, left_wheel.Target_Position, left_wheel.Reality_Position, PWMA1, PWMA2);
//        ano_send_user_data(1, left_wheel.Target_Velocity, left_wheel.Reality_Velocity, PWMA1, PWMA2);
//        ano_send_user_data(1, right_wheel.Target_Velocity, right_wheel.Reality_Velocity, PWMB1, PWMB2);
        ano_send_user_data(1, left_wheel.Target_Position, left_wheel.Reality_Position, left_wheel.Target_Velocity, left_wheel.Reality_Velocity);
        ano_send_user_data(2, PWMA1, PWMA2, 0, 0);

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
    timeout_s.usec = 10000;
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    return ret;
}
