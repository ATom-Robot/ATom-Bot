/*
 * qboot_gd32.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2023-07-17     qiyongzhong       first version
 */

#include <board.h>

#ifdef SOC_FAMILY_GD32

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


void hal_DeInit(void)
{
    //AHB1 peripherals 
    rcu_periph_reset_enable(RCU_GPIOARST);    
    rcu_periph_reset_enable(RCU_GPIOBRST);   
    rcu_periph_reset_enable(RCU_GPIOCRST);   
    rcu_periph_reset_enable(RCU_GPIODRST);  
    rcu_periph_reset_enable(RCU_GPIOERST);   
    rcu_periph_reset_enable(RCU_GPIOFRST);   
    rcu_periph_reset_enable(RCU_GPIOGRST);  
    rcu_periph_reset_enable(RCU_GPIOHRST);  
    rcu_periph_reset_enable(RCU_GPIOIRST);   
    rcu_periph_reset_enable(RCU_CRCRST);     
    rcu_periph_reset_enable(RCU_DMA0RST);   
    rcu_periph_reset_enable(RCU_DMA1RST);   
    rcu_periph_reset_enable(RCU_IPARST);     
    rcu_periph_reset_enable(RCU_ENETRST);    
    rcu_periph_reset_enable(RCU_USBHSRST);  
    //AHB2 peripheral
    rcu_periph_reset_enable(RCU_DCIRST);     
    rcu_periph_reset_enable(RCU_TRNGRST);   
    rcu_periph_reset_enable(RCU_USBFSRST);   
    //AHB3 peripheral
    rcu_periph_reset_enable(RCU_EXMCRST);   
    // APB1 peripheral
    rcu_periph_reset_enable(RCU_TIMER1RST);  
    rcu_periph_reset_enable(RCU_TIMER2RST);  
    rcu_periph_reset_enable(RCU_TIMER3RST);  
    rcu_periph_reset_enable(RCU_TIMER4RST);  
    rcu_periph_reset_enable(RCU_TIMER5RST);  
    rcu_periph_reset_enable(RCU_TIMER6RST);  
    rcu_periph_reset_enable(RCU_TIMER11RST); 
    rcu_periph_reset_enable(RCU_TIMER12RST); 
    rcu_periph_reset_enable(RCU_TIMER13RST); 
    rcu_periph_reset_enable(RCU_WWDGTRST); 
    rcu_periph_reset_enable(RCU_SPI1RST);     
    rcu_periph_reset_enable(RCU_SPI2RST);     
    rcu_periph_reset_enable(RCU_USART1RST); 
    rcu_periph_reset_enable(RCU_USART2RST); 
    rcu_periph_reset_enable(RCU_UART3RST);  
    rcu_periph_reset_enable(RCU_UART4RST);  
    rcu_periph_reset_enable(RCU_I2C0RST);    
    rcu_periph_reset_enable(RCU_I2C1RST);    
    rcu_periph_reset_enable(RCU_I2C2RST);    
    rcu_periph_reset_enable(RCU_CAN0RST);   
    rcu_periph_reset_enable(RCU_CAN1RST);   
    rcu_periph_reset_enable(RCU_PMURST);    
    rcu_periph_reset_enable(RCU_DACRST);    
    rcu_periph_reset_enable(RCU_UART6RST);  
    rcu_periph_reset_enable(RCU_UART7RST);  
    //APB2 peripheral
    rcu_periph_reset_enable(RCU_TIMER0RST);  
    rcu_periph_reset_enable(RCU_TIMER7RST);  
    rcu_periph_reset_enable(RCU_USART0RST); 
    rcu_periph_reset_enable(RCU_USART5RST); 
    rcu_periph_reset_enable(RCU_ADCRST);    
    rcu_periph_reset_enable(RCU_SDIORST);    
    rcu_periph_reset_enable(RCU_SPI0RST);     
    rcu_periph_reset_enable(RCU_SPI3RST);     
    rcu_periph_reset_enable(RCU_SYSCFGRST);  
    rcu_periph_reset_enable(RCU_TIMER8RST);  
    rcu_periph_reset_enable(RCU_TIMER9RST);  
    rcu_periph_reset_enable(RCU_TIMER10RST); 
    rcu_periph_reset_enable(RCU_SPI4RST);     
    rcu_periph_reset_enable(RCU_SPI5RST);     
    rcu_periph_reset_enable(RCU_TLIRST);      
    //APB1 additional 
    rcu_periph_reset_enable(RCU_CTCRST);     
    rcu_periph_reset_enable(RCU_IREFRST); 

//AHB1 peripherals 
    rcu_periph_reset_disable(RCU_GPIOARST);    
    rcu_periph_reset_disable(RCU_GPIOBRST);   
    rcu_periph_reset_disable(RCU_GPIOCRST);   
    rcu_periph_reset_disable(RCU_GPIODRST);  
    rcu_periph_reset_disable(RCU_GPIOERST);   
    rcu_periph_reset_disable(RCU_GPIOFRST);   
    rcu_periph_reset_disable(RCU_GPIOGRST);  
    rcu_periph_reset_disable(RCU_GPIOHRST);  
    rcu_periph_reset_disable(RCU_GPIOIRST);   
    rcu_periph_reset_disable(RCU_CRCRST);     
    rcu_periph_reset_disable(RCU_DMA0RST);   
    rcu_periph_reset_disable(RCU_DMA1RST);   
    rcu_periph_reset_disable(RCU_IPARST);     
    rcu_periph_reset_disable(RCU_ENETRST);    
    rcu_periph_reset_disable(RCU_USBHSRST);  
    //AHB2 peripheral
    rcu_periph_reset_disable(RCU_DCIRST);     
    rcu_periph_reset_disable(RCU_TRNGRST);   
    rcu_periph_reset_disable(RCU_USBFSRST);   
    //AHB3 peripheral
    rcu_periph_reset_disable(RCU_EXMCRST);   
    // APB1 peripheral
    rcu_periph_reset_disable(RCU_TIMER1RST);  
    rcu_periph_reset_disable(RCU_TIMER2RST);  
    rcu_periph_reset_disable(RCU_TIMER3RST);  
    rcu_periph_reset_disable(RCU_TIMER4RST);  
    rcu_periph_reset_disable(RCU_TIMER5RST);  
    rcu_periph_reset_disable(RCU_TIMER6RST);  
    rcu_periph_reset_disable(RCU_TIMER11RST); 
    rcu_periph_reset_disable(RCU_TIMER12RST); 
    rcu_periph_reset_disable(RCU_TIMER13RST); 
    rcu_periph_reset_disable(RCU_WWDGTRST); 
    rcu_periph_reset_disable(RCU_SPI1RST);     
    rcu_periph_reset_disable(RCU_SPI2RST);     
    rcu_periph_reset_disable(RCU_USART1RST); 
    rcu_periph_reset_disable(RCU_USART2RST); 
    rcu_periph_reset_disable(RCU_UART3RST);  
    rcu_periph_reset_disable(RCU_UART4RST);  
    rcu_periph_reset_disable(RCU_I2C0RST);    
    rcu_periph_reset_disable(RCU_I2C1RST);    
    rcu_periph_reset_disable(RCU_I2C2RST);    
    rcu_periph_reset_disable(RCU_CAN0RST);   
    rcu_periph_reset_disable(RCU_CAN1RST);   
    rcu_periph_reset_disable(RCU_PMURST);    
    rcu_periph_reset_disable(RCU_DACRST);    
    rcu_periph_reset_disable(RCU_UART6RST);  
    rcu_periph_reset_disable(RCU_UART7RST);  
    //APB2 peripheral
    rcu_periph_reset_disable(RCU_TIMER0RST);  
    rcu_periph_reset_disable(RCU_TIMER7RST);  
    rcu_periph_reset_disable(RCU_USART0RST); 
    rcu_periph_reset_disable(RCU_USART5RST); 
    rcu_periph_reset_disable(RCU_ADCRST);    
    rcu_periph_reset_disable(RCU_SDIORST);    
    rcu_periph_reset_disable(RCU_SPI0RST);     
    rcu_periph_reset_disable(RCU_SPI3RST);     
    rcu_periph_reset_disable(RCU_SYSCFGRST);  
    rcu_periph_reset_disable(RCU_TIMER8RST);  
    rcu_periph_reset_disable(RCU_TIMER9RST);  
    rcu_periph_reset_disable(RCU_TIMER10RST); 
    rcu_periph_reset_disable(RCU_SPI4RST);     
    rcu_periph_reset_disable(RCU_SPI5RST);     
    rcu_periph_reset_disable(RCU_TLIRST);      
    //APB1 additional 
    rcu_periph_reset_disable(RCU_CTCRST);     
    rcu_periph_reset_disable(RCU_IREFRST); 
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
    hal_DeInit();

    for(int i=0; i<128; i++)
    {
        NVIC_DisableIRQ(i);
        NVIC_ClearPendingIRQ(i);
    }

    //Resets the RCC clock configuration to the default reset state.
    rcu_deinit();   

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    __set_CONTROL(0);
    __set_MSP(stk_addr);

    app_func();//Jump to application running

    LOG_E("Qboot jump to application fail.");
}
#endif


