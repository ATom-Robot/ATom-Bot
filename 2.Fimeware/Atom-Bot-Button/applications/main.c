/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66         First version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "BSP_Joint.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"

#define POWER_SW GET_PIN(C, 6)
#define VOLTAGE_ADC_EN GET_PIN(B, 12)

int main(void)
{
    rt_kprintf("Hello Atom-Bot!!\n");

    rt_kprintf("SCL:%d,SDA:%d\n", GET_PIN(B, 10), GET_PIN(B, 11));

    rt_kprintf("LEFT_ENCODER_PIN:%d  RIGHT_ENCODER_PIN:%d\n", GET_PIN(A, 0), GET_PIN(A, 1));
    rt_kprintf("LEFT_ENCODER_PIN:%d  RIGHT_ENCODER_PIN:%d\n", GET_PIN(A, 6), GET_PIN(A, 7));

    MX_TIM4_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    Motor_Init();
    Encoder_Init();

    joint_i2s_init();

    rt_pin_mode(POWER_SW, PIN_MODE_OUTPUT);
    rt_pin_mode(VOLTAGE_ADC_EN, PIN_MODE_OUTPUT);
    rt_pin_write(VOLTAGE_ADC_EN, PIN_LOW);

//  MOTOR_SetSpeed(1, 500);
//  MOTOR_SetSpeed(2, 500);

    while (1)
    {
        rt_pin_write(POWER_SW, PIN_HIGH);
        rt_pin_write(POWER_SW, PIN_LOW);
        rt_thread_mdelay(200);
    }

    return 0;
}

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

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
    /* USER CODE BEGIN TIM4_IRQn 0 */
    rt_interrupt_enter();
    /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4);
    /* USER CODE BEGIN TIM4_IRQn 1 */
    rt_interrupt_leave();
    /* USER CODE END TIM4_IRQn 1 */
}
