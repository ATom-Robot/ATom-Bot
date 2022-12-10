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
	
	extern void MX_TIM4_Init(void);
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


#define PWM_DEV_NAME        "pwm4"		/* PWM设备名称 */
#define PWM_DEV_CHANNEL1     1			/* PWM通道 */
#define PWM_DEV_CHANNEL2     2			/* PWM通道 */

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
	

    return RT_EOK;

}
MSH_CMD_EXPORT(car_pwmInit, car_pwmInit);

rt_int8_t car_speedSet(int argc, const char**argv)
{
	if (argc > 0)
	{
		rt_kprintf("set speed:%d\n", atoi(argv[1]));
		rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL1, 1000 * 1000, atoi(argv[1])); // 500 * 1000
		rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL2, 1000 * 1000, 0);		
	}	
	
	return RT_EOK;
}
MSH_CMD_EXPORT(car_speedSet, car set speed);
