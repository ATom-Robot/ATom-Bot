/*
 * qboot_stm32.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-08-31     qiyongzhong       first version
 */

#include <board.h>

#ifdef CHIP_FAMILY_STM32

#include <rtthread.h>
#include <rtdevice.h>
#include <qboot.h>

//#define QBOOT_APP_RUN_IN_QSPI_FLASH
#define QBOOT_QSPI_FLASH_DEVICE_NAME    "norspi"

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
static int nor_flash_memory_mapped_mode(struct rt_qspi_device * device)
{
    QSPI_CommandTypeDef s_command;
    QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

    /* Configure the command for the read instruction */
    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = 0xEB;
    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
    s_command.AlternateBytes    = 0x00;
    s_command.AlternateBytesSize= QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.DataMode          = QSPI_DATA_4_LINES;
    s_command.DummyCycles       = 4;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Configure the memory mapped mode */
    s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_ENABLE;
    s_mem_mapped_cfg.TimeOutPeriod     = 1;

    if (HAL_QSPI_MemoryMapped((QSPI_HandleTypeDef *)(device->parent.bus->parent.user_data), &s_command, &s_mem_mapped_cfg) != HAL_OK)
    {
        rt_kprintf("memory mapped mode failed\n");
        return -RT_ERROR;
    }
    
    return RT_EOK;

}

void qbt_jump_to_app(void)
{
    typedef void (*app_func_t)(void);
    app_func_t app_func;
    struct rt_qspi_device *device = NULL;
    u32 stk_addr = 0;
    u32 reset_handler = 0;
    const u8 send[4] = {0x03, 0x00, 0x00, 0x00};
    u8 recv[8] = {0};
    
    device = (struct rt_qspi_device *)rt_device_find(QBOOT_QSPI_FLASH_DEVICE_NAME);

    if(device == RT_NULL)
    {
        LOG_E("no found qspi flash.");
        return;
    }

    rt_qspi_send_then_recv(device, send, sizeof(send), recv, sizeof(recv));

    stk_addr = recv[0];
    stk_addr += ((u32)recv[1] << 8);
    stk_addr += ((u32)recv[2] << 16);
    stk_addr += ((u32)recv[3] << 24);

    reset_handler = recv[4];
    reset_handler += ((u32)recv[5] << 8);
    reset_handler += ((u32)recv[6] << 16);
    reset_handler += ((u32)recv[7] << 24);

    LOG_D("stk_addr :\t\t0x%08X", stk_addr);
    LOG_D("reset_handler :\t0x%08X",reset_handler);

    stk_addr &= 0x3FF00000;
    if(((reset_handler & 0xff000000) != QSPI_BASE) || 
        (stk_addr != 0x20000000 && stk_addr != 0x24000000 && 
            stk_addr != 0x30000000 && stk_addr != 0x38000000))
    {
        LOG_E("No legitimate application.");
        return;
    }

    if(nor_flash_memory_mapped_mode(device) != RT_EOK)
    {
        return;
    }

    rt_kprintf("Jump to application running ... \n");

    __disable_irq();

    #if (__MPU_PRESENT == 1)
    HAL_MPU_Disable();
    #endif
    
    SCB_DisableICache();
    SCB_DisableDCache();
    
    __set_CONTROL(0);
    __set_MSP(stk_addr);
    
    app_func = (app_func_t) ((__IO uint32_t) (QSPI_BASE + 4));
    app_func();

    LOG_E("Qboot jump to application fail.");
}
#else
void qbt_jump_to_app(void)
{
    typedef void (*app_func_t)(void);
    u32 app_addr = QBOOT_APP_ADDR;
    u32 stk_addr = *((__IO uint32_t *)app_addr);
    app_func_t app_func = (app_func_t)(*((__IO uint32_t *)(app_addr + 4)));

    if ((((u32)app_func & 0xff000000) != 0x08000000) || (((stk_addr & 0x2ff00000) != 0x20000000) && ((stk_addr & 0x2ff00000) != 0x24000000)))
    {
        LOG_E("No legitimate application.");
        return;
    }

    rt_kprintf("Jump to application running ... \n");
    rt_thread_mdelay(200);
    
    __disable_irq();
    HAL_DeInit();
    HAL_RCC_DeInit();
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for(int i=0; i<128; i++)
    {
        HAL_NVIC_DisableIRQ(i);
        HAL_NVIC_ClearPendingIRQ(i);
    }
    
    __set_CONTROL(0);
    __set_MSP(stk_addr);
    
    app_func();//Jump to application running
    
    LOG_E("Qboot jump to application fail.");
}
#endif
#endif

