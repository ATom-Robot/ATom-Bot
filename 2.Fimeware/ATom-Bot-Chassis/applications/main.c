/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb666        First version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#ifndef RT_USING_FAL
    #include "fal_cfg.h"
#else
    #include "fal.h"
#endif
#include "BSP_Joint.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_pid.h"
#include "drv_sensor.h"
#include "ano.h"

#define APP_VERSION     "1.1.0"

#define POWER_SW        GET_PIN(C, 6)
#define VOLTAGE_ADC_EN  GET_PIN(B, 12)

extern int control_hwtimer(void);
extern void app_motion_ctrl_init(void);
extern int MPU6050_Init(void);
extern int read_mpu6xxx_td(void);
extern int vl53l0x_port(void);

int main(void)
{
    rt_kprintf("Hello ATom-Bot!!\n");

    rt_kprintf(" ______  ______                              ____            __      \n");
    rt_kprintf("/\\  _  \\/\\__  _\\                            /\\  _`\\         /\\ \\__   \n");
    rt_kprintf("\\ \\ \\L\\ \\/_/\\ \\/   ___     ___ ___          \\ \\ \\L\\ \\    ___\\ \\ ,_\\  \n");
    rt_kprintf(" \\ \\  __ \\ \\ \\ \\  / __`\\ /' __` __`\\  _______\\ \\  _ <'  / __`\\ \\ \\/  \n");
    rt_kprintf("  \\ \\ \\/\\ \\ \\ \\ \\/\\ \\L\\ \\/\\ \\/\\ \\/\\ \\/\\______\\\\ \\ \\L\\ \\/\\ \\L\\ \\ \\ \\_ \n");
    rt_kprintf("   \\ \\_\\ \\_\\ \\ \\_\\ \\____/\\ \\_\\ \\_\\ \\_\\/______/ \\ \\____/\\ \\____/\\ \\__\\ \n");
    rt_kprintf("    \\/_/\\/_/  \\/_/\\/___/  \\/_/\\/_/\\/_/          \\/___/  \\/___/  \\/__/ \n");

    rt_kprintf("--------------------------------------------------------\n");
    rt_kprintf("Current version of app fimeware is:%s\n", APP_VERSION);
    rt_kprintf("--------------------------------------------------------\n");

    rt_kprintf("SCL1:%d,SDA1:%d\n", GET_PIN(A, 3), GET_PIN(B, 1));
    rt_kprintf("SCL2:%d,SDA2:%d\n", GET_PIN(B, 10), GET_PIN(B, 11));

    rt_kprintf("LEFT_ENCODER_PIN:%d  RIGHT_ENCODER_PIN:%d\n", GET_PIN(A, 0), GET_PIN(A, 1));
    rt_kprintf("LEFT_ENCODER_PIN:%d  RIGHT_ENCODER_PIN:%d\n", GET_PIN(A, 6), GET_PIN(A, 7));

    rt_pin_mode(POWER_SW, PIN_MODE_OUTPUT);
    rt_pin_mode(VOLTAGE_ADC_EN, PIN_MODE_OUTPUT);
    rt_pin_write(VOLTAGE_ADC_EN, PIN_LOW);
#ifdef RT_USING_FAL
    fal_init();
#endif
    MX_TIM4_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    Motor_Init();
    Encoder_Init();
    MPU6050_Init();
    vl53l0x_port();

    joint_i2c_init();
    ano_init("uart3");

    sensor_thread_create();

    /* PID Init*/
    Loc_level_PID_Init();
    control_hwtimer();
    app_motion_ctrl_init();

    while (1)
    {
        rt_pin_write(POWER_SW, PIN_HIGH);
        rt_pin_write(POWER_SW, PIN_LOW);
        rt_thread_mdelay(200);
    }

    return 0;
}

/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
static int ota_app_vtor_reconfig(void)
{
#define NVIC_VTOR_MASK   0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    rt_interrupt_enter();
    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */
    rt_interrupt_leave();
    /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    rt_interrupt_enter();
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    rt_interrupt_leave();
    /* USER CODE END TIM3_IRQn 1 */
}
