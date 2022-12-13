/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

int main(void)
{
    rt_kprintf("Hello Atom-Bot!!\n");

    rt_kprintf("SCL:%d,SDA:%d\n", GET_PIN(B, 10), GET_PIN(B, 11));

    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    return 0;
}

#define MOTOR_IN1 GET_PIN(B, 6)
#define MOTOR_IN2 GET_PIN(B, 7)

int motor_run_test()
{
    rt_pin_mode(MOTOR_IN1, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_IN2, PIN_MODE_OUTPUT);

    rt_pin_write(MOTOR_IN1, PIN_HIGH);
    rt_pin_write(MOTOR_IN2, PIN_LOW);

    rt_kprintf("MOTOR_IN1:%d	MOTOR_IN2:%d\n", rt_pin_read(MOTOR_IN1), rt_pin_read(MOTOR_IN2));
    return 0;
}
MSH_CMD_EXPORT(motor_run_test, motor_run_test);


#define PWM_DEV_NAME        "pwm4"      /* PWM设备名称 */
#define PWM_DEV_CHANNEL1     1          /* PWM通道 */
#define PWM_DEV_CHANNEL2     2          /* PWM通道 */
#define PWM_DEV_CHANNER1     3          /* PWM通道 */
#define PWM_DEV_CHANNER2     4          /* PWM通道 */

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

rt_int8_t car_pwmInit(void)
{
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("\r\npwm run failed! can't find %s device!\r\n", PWM_DEV_NAME);
        return RT_ERROR;
    }
    rt_kprintf("\r\npwm run success! find %s device!\r\n", PWM_DEV_NAME);

    //开启PWM
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL1);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL2);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNER1);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNER2);

    return RT_EOK;

}
MSH_CMD_EXPORT(car_pwmInit, car_pwmInit);

rt_int8_t car_speedSet_L(int argc, const char **argv)
{
    if (argc > 1)
    {
        rt_kprintf("set speed_L:%d\n", atoi(argv[1]));
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL1, 1000 * 1000, atoi(argv[1])); // 500 * 1000
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL2, 1000 * 1000, 0);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(car_speedSet_L, car set speed right);

rt_int8_t car_speedSet_R(int argc, const char **argv)
{
    if (argc > 1)
    {
        rt_kprintf("set speed_R:%d\n", atoi(argv[1]));
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNER1, 1000 * 1000, atoi(argv[1])); // 500 * 1000
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNER2, 1000 * 1000, 0);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(car_speedSet_R, car set speed left);

int hw_Encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    return RT_EOK;
}
MSH_CMD_EXPORT(hw_Encoder_init, Encoder_init);

void encoder_clearCounter(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

#define PULSE_ENCODER_R_DEV_NAME    "pulse2"    /* 脉冲编码器名称 */
#define PULSE_ENCODER_L_DEV_NAME    "pulse3"    /* 脉冲编码器名称 */

static int pulse_encoder_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_device_t pulse_encoderR_dev = RT_NULL;   /* 脉冲编码器设备句柄 */
    rt_device_t pulse_encoderL_dev = RT_NULL;   /* 脉冲编码器设备句柄 */

    rt_uint32_t index;
    rt_int32_t count_R;
    rt_int32_t count_L;

    /* 查找脉冲编码器设备 */
    pulse_encoderR_dev = rt_device_find(PULSE_ENCODER_R_DEV_NAME);
    if (pulse_encoderR_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_R_DEV_NAME);
        return RT_ERROR;
    }
    pulse_encoderL_dev = rt_device_find(PULSE_ENCODER_L_DEV_NAME);
    if (pulse_encoderL_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_L_DEV_NAME);
        return RT_ERROR;
    }

    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoderR_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_R_DEV_NAME);
        return ret;
    }
    ret = rt_device_open(pulse_encoderL_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_L_DEV_NAME);
        return ret;
    }

    for (index = 0; index <= 30; index ++)
    {
        rt_thread_mdelay(200);
        /* 读取脉冲编码器计数值 */
        rt_device_read(pulse_encoderR_dev, 0, &count_R, 1);
        rt_device_read(pulse_encoderL_dev, 0, &count_L, 1);
        /* 清空脉冲编码器计数值 */
        rt_device_control(pulse_encoderR_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        rt_device_control(pulse_encoderL_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);

        rt_kprintf("=================================\n");
        rt_kprintf("R count:%d   |   L count:%d\n", count_R, count_L);
    }

    rt_device_close(pulse_encoderR_dev);
    rt_device_close(pulse_encoderL_dev);

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(pulse_encoder_sample, pulse encoder sample);