/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-20     Cheung       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_i2c.h"

#ifdef BSP_USING_HW_I2C

#define DBG_TAG              "drv.i2c"
#ifdef DRV_DEBUG
    #define DBG_LVL               DBG_LOG
#else
    #define DBG_LVL               DBG_INFO
#endif /* DRV_DEBUG */
#include <rtdbg.h>

/** @defgroup Private Defines
  * @{
  */
#define HAED_I2C_CONFIG(x)                  \
{                                           \
    .bus_name = "i2c"#x,                    \
    .scl_pin = BSP_I2C##x##_SCL_PIN,        \
    .sda_pin = BSP_I2C##x##_SDA_PIN,        \
    .speed = BSP_I2C##x##_CLOCK,            \
    .pInitFunc = MX_I2C##x##_Init,          \
    .pHi2c = &hi2c##x,                      \
    .i2c_bus = {                            \
        .ops = &i2c_bus_ops,                \
    },                                      \
}

/** @defgroup Private Variables
  * @{
  */
#ifdef BSP_USING_HW_I2C2
    static I2C_HandleTypeDef hi2c2;
#endif /* BSP_USING_HW_I2C2 */
#ifdef BSP_USING_HW_I2C3
    static I2C_HandleTypeDef hi2c3;
#endif /* BSP_USING_HW_I2C3 */

/** @defgroup Private Functions
  * @{
  */
static rt_size_t i2c_xfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg msgs[],
                          rt_uint32_t num);
#ifdef BSP_USING_HW_I2C2
    static void MX_I2C2_Init(rt_uint32_t speed);
#endif /* BSP_USING_HW_I2C2 */
#ifdef BSP_USING_HW_I2C3
    static void MX_I2C3_Init(rt_uint32_t speed)
#endif /* BSP_USING_HW_I2C3 */

static struct rt_i2c_bus_device_ops i2c_bus_ops =
{
    .master_xfer = i2c_xfer,
    .slave_xfer = RT_NULL,
    .i2c_bus_control = RT_NULL,
};

static struct stm32_hard_i2c_config hard_i2c_config[] =
{
#ifdef BSP_USING_HW_I2C2
    HAED_I2C_CONFIG(2),
#endif /* BSP_USING_HW_I2C2 */

#ifdef BSP_USING_HW_I2C3
    HAED_I2C_CONFIG(3)
#endif /* BSP_USING_HW_I2C3 */
};

enum
{
#ifdef BSP_USING_HW_I2C2
    I2C2_INDEX,
#endif /* BSP_USING_HW_I2C2 */

#ifdef BSP_USING_HW_I2C3
    I2C3_INDEX,
#endif /* BSP_USING_HW_I2C3 */
};

/**
 * I2C bus common interrupt process. This need add to I2C ISR.
 *
 * @param config hard i2c bus instance pointor
 */
static inline void i2cx_event_isr(struct stm32_hard_i2c_config *config)
{
    HAL_I2C_EV_IRQHandler(config->pHi2c);
}
static inline void i2cx_error_isr(struct stm32_hard_i2c_config *config)
{
    HAL_I2C_ER_IRQHandler(config->pHi2c);
}

/**
 * if i2c is locked, this function will unlock it
 *
 * @param stm32 config class
 *
 * @return RT_EOK indicates successful unlock.
 */
static rt_err_t hard_i2c_bus_unlock(struct stm32_hard_i2c_config *config)
{
    rt_int32_t i = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_TypeDef *scl_port;
    GPIO_TypeDef *sda_port;

    if (PIN_LOW == rt_pin_read(config->sda_pin))
    {
        while (i++ < 9)
        {
            HAL_GPIO_WritePin(scl_port, config->scl_pin, PIN_HIGH);
            rt_hw_us_delay(100);
            HAL_GPIO_WritePin(scl_port, config->scl_pin, PIN_LOW);
            rt_hw_us_delay(100);
        }
    }

    if (PIN_LOW == rt_pin_read(config->sda_pin))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

#ifdef BSP_USING_HW_I2C2
static void MX_I2C2_Init(rt_uint32_t speed)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = speed;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    /* NVIC for I2Cx */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

void I2C2_EV_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    i2cx_event_isr(&hard_i2c_config[I2C2_INDEX]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void I2C2_ER_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    i2cx_error_isr(&hard_i2c_config[I2C2_INDEX]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* release sem to notice function */
    rt_completion_done(&hard_i2c_config[I2C2_INDEX].tx_notice);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* release sem to notice function */
    rt_completion_done(&hard_i2c_config[I2C2_INDEX].rx_notice);
}
#endif

/**
 * @brief  i2c bus data xfer for transmit and receive.
 * @param  bus pointer to hard i2c configurate instance.
 *         msgs info data to xfer by i2c bus.
 *         num the total data length to xfer.
 * @retval function execute result if success is xfer count.
 */
static rt_size_t i2c_xfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg msgs[],
                          rt_uint32_t num)
{
    HAL_StatusTypeDef status;
    rt_err_t ret = 0;
    rt_size_t xfer_len = 0;
    struct rt_i2c_msg *msg;
    struct stm32_hard_i2c_config *pCfg = rt_container_of(bus, struct stm32_hard_i2c_config, i2c_bus);

    for (int i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags & RT_I2C_RD)
        {
            /***** I2C master to receive *****/

            /* master receive data by interrupt */
            status = HAL_I2C_Master_Receive_IT(pCfg->pHi2c, msg->addr << 1, msg->buf, msg->len);
            if (status != HAL_OK)
            {
                HAL_I2C_Master_Abort_IT(pCfg->pHi2c, msg->addr << 1);
                ret = RT_ERROR;
                goto __exit;
            }

            /* wait receive complete */
            ret = rt_completion_wait(&pCfg->rx_notice, (10 * msg->len));
            if (ret != RT_EOK)
            {
                HAL_I2C_Master_Abort_IT(pCfg->pHi2c, msg->addr << 1);
                goto __exit;
            }

            xfer_len++;
        }
        else
        {
            /***** I2C master to transmit *****/

            /* master transmit data by interrupt */
            status = HAL_I2C_Master_Transmit_IT(pCfg->pHi2c, msg->addr << 1, msg->buf, msg->len);
            if (status != HAL_OK)
            {
                HAL_I2C_Master_Abort_IT(pCfg->pHi2c, msg->addr << 1);
                ret = RT_ERROR;
                goto __exit;
            }

            /* wait transmit complete */
            ret = rt_completion_wait(&pCfg->tx_notice, (10 * msg->len));
            if (ret != RT_EOK)
            {
                HAL_I2C_Master_Abort_IT(pCfg->pHi2c, msg->addr << 1);
                goto __exit;
            }

            xfer_len++;
        }
    }

__exit:
    if (xfer_len == num)
    {
        ret = xfer_len;
    }

    return ret;
}

/* Hard I2C initialization function */
int rt_hw_hardi2c_init(void)
{
    int result = RT_ERROR;
    char name[RT_NAME_MAX];
    rt_size_t obj_num = sizeof(hard_i2c_config) / sizeof(struct stm32_hard_i2c_config);

    for (int i = 0; i < obj_num; i++)
    {
        /* Step 1: start to init i2c bus GPIO to OD mode */

        /* Step 2: To check SDA is or not pull-down, when is pull-down to send 9-clocks unlock bus */
        hard_i2c_bus_unlock(&hard_i2c_config[i]);

        /* Step 3: To configurate hard I2C bus by speed */
        hard_i2c_config[i].pInitFunc(hard_i2c_config[i].speed);

        /* Step 4: register bus to kernel */
        result = rt_i2c_bus_device_register(&(hard_i2c_config[i].i2c_bus), hard_i2c_config[i].bus_name);
        if (result != RT_EOK)
        {
            LOG_E("%s bus init failed!", hard_i2c_config[i].bus_name);
            result |= RT_ERROR;
        }
        else
        {
            LOG_I("%s bus init success!", hard_i2c_config[i].bus_name);
            result |= RT_EOK;
        }

        /* Step 5: create mutex and notice semaphore */
        if (result == RT_EOK)
        {
            /* tx sem notice create */
            rt_memset(name, 0, RT_NAME_MAX);
            rt_snprintf(name, RT_NAME_MAX, "%s_t", hard_i2c_config[i].bus_name);
            rt_completion_init(&hard_i2c_config[i].tx_notice);

            /* rx sem notice create */
            rt_memset(name, 0, RT_NAME_MAX);
            rt_snprintf(name, RT_NAME_MAX, "%s_r", hard_i2c_config[i].bus_name);
            rt_completion_init(&hard_i2c_config[i].rx_notice);
        }
    }

    return result;
}
INIT_BOARD_EXPORT(rt_hw_hardi2c_init);
#endif
