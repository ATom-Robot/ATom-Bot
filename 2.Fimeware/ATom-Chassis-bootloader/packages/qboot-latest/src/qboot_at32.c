/*
 * qboot_stm32.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-08-31     qiyongzhong       first version
 */

#include <board.h>

#ifdef CHIP_FAMILY_AT32

#include <rtthread.h>
#include <rtdevice.h>
#include <qboot.h>

//#define QBOOT_APP_RUN_IN_QSPI_FLASH
//#define QBOOT_DEBUG
#define QBOOT_USING_LOG
#define DBG_TAG "Qboot"

#ifdef QBOOT_DEBUG
#define DBG_LVL DBG_LOG
#else
#define DBG_LVL DBG_INFO
#endif

#ifdef QBOOT_USING_LOG
#ifndef DBG_ENABLE
#define DBG_ENABLE
#endif
#ifndef DBG_COLOR
#define DBG_COLOR
#endif
#endif

#include <rtdbg.h>

#ifdef QBOOT_APP_RUN_IN_QSPI_FLASH

static void qbt_qspi_flash_init(void)
{
    //waiting realize
}

void qbt_jump_to_app(void)
{
    qbt_qspi_flash_init();
    
    //waiting realize
}

#else

static void qbt_reset_periph(void)
{
    RCC->AHBRST  = 0xFFFFFFFF;
    RCC->AHBRST  = 0x00000000;
    
    RCC->APB1RST = 0xFFFFFFFF;
    RCC->APB1RST = 0x00000000;
    
    RCC->APB2RST = 0xFFFFFFFF;
    RCC->APB2RST = 0x00000000;
}

void qbt_jump_to_app(void)
{
    typedef void (*app_func_t)(void);
    u32 app_addr = QBOOT_APP_ADDR;
    u32 stk_addr = *((__IO uint32_t *)app_addr);
    app_func_t app_func = (app_func_t)(*((__IO uint32_t *)(app_addr + 4)));

    if ((((u32)app_func & 0xff000000) != 0x08000000) || ((stk_addr & 0x2ff00000) != 0x20000000))
    {
        LOG_E("No legitimate application.");
        return;
    }

    rt_kprintf("Jump to application running ... \n");
    rt_thread_mdelay(200);
    
    __disable_irq();
    qbt_reset_periph();

    for(int i=0; i<128; i++)
    {
        __NVIC_DisableIRQ(i);
        __NVIC_ClearPendingIRQ(i);
    }
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    RCC_Reset();
    
    __set_CONTROL(0);
    __set_MSP(stk_addr);
    
    app_func();//Jump to application running
    
    LOG_E("Qboot jump to application fail.");
}
#endif
#endif

