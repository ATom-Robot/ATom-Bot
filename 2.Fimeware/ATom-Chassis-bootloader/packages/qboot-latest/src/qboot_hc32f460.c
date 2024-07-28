/*
 * qboot_hc32f460.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-08-31     qiyongzhong       first version
 */

#include <board.h>

#ifdef CHIP_FAMILY_HC32

#include <rtthread.h>
#include <rtdevice.h>
#include <qboot.h>
#include <hc32f460.h>
#include "hc32_ll.h"

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
    #define DEFAULT_FCG0                        (0xFFFFFAEEul)
    #define DEFAULT_FCG1                        (0xFFFFFFFFul)
    #define DEFAULT_FCG2                        (0xFFFFFFFFul)
    #define DEFAULT_FCG3                        (0xFFFFFFFFul)

    PWC_FCG0_REG_Unlock();

    CM_PWC->FCG0 = DEFAULT_FCG0;
    CM_PWC->FCG1 = DEFAULT_FCG1;
    CM_PWC->FCG2 = DEFAULT_FCG2;
    CM_PWC->FCG3 = DEFAULT_FCG3;

    PWC_FCG0_REG_Lock();
}

void qbt_jump_to_app(void)
{
    typedef void (*app_func_t)(void);
    u32 app_addr = QBOOT_APP_ADDR;
    u32 stk_addr = *((__IO uint32_t *)app_addr);
    app_func_t app_func = (app_func_t)(*((__IO uint32_t *)(app_addr + 4)));

    if ((((u32)app_func & 0xff000000) != 0x00000000) || (((stk_addr+0x00010000) & 0x2ff00000) != 0x20000000))
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
        NVIC_DisableIRQ(i);
        NVIC_ClearPendingIRQ(i);
        //enIrqResign(i);
    }
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    CLK_MrcCmd(ENABLE);
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_MRC);
    EFM_FWMC_Cmd(ENABLE);
    EFM_SetWaitCycle(EFM_WAIT_CYCLE0);
    EFM_FWMC_Cmd(DISABLE);
    
    __set_CONTROL(0);
    __set_MSP(stk_addr);
    
    app_func();//Jump to application running
    
    LOG_E("Qboot jump to application fail.");
}
#endif
#endif

