#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <dfs_fs.h>
#include "dfs_romfs.h"
#include <fal.h>
#include "led.h"
#include "internal_web.h"
#include "boot.h"

#define BUTTON_PIN          GET_PIN(E, 1)

#define ENTER_BOOT_TICK     5000
#define RECOVERY_TICK       10000

static int boot_check_key(void)
{
    rt_tick_t enter_tick = 0;
    rt_tick_t press_tick = 0;

    rt_pin_mode(BUTTON_PIN, PIN_MODE_INPUT);

    rt_thread_mdelay(500);

    enter_tick = rt_tick_get();
    press_tick = 0;
    while(rt_pin_read(BUTTON_PIN) == PIN_LOW)
    {
        rt_thread_mdelay(100);
        press_tick = rt_tick_get() - enter_tick;
        
        if(press_tick >= RECOVERY_TICK)
            led_recovery_mode();
        else if(press_tick >= ENTER_BOOT_TICK)
            led_boot_ready_mode();
    }

    if(press_tick >= RECOVERY_TICK)
    {
        __disable_irq();
        RTC_HandleTypeDef RTC_Handler;
        RTC_Handler.Instance = RTC;
        HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR1, 0xA5A5);
        HAL_NVIC_SystemReset();
    }
    
    if(press_tick < ENTER_BOOT_TICK)
    {
        int result = boot_act();
        if(result == RT_EOK)
        {
            __disable_irq();
            RTC_HandleTypeDef RTC_Handler;
            RTC_Handler.Instance = RTC;
            HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR0, 0xA5A5);
            HAL_NVIC_SystemReset();
        }
        else
        {
            __disable_irq();
            HAL_NVIC_SystemReset();
        }
    }

    return RT_EOK;
}

extern int phy_init(void);
extern int rt_hw_stm32_eth_init(void);
extern int telnet_module_init(void);
extern int finsh_system_init(void);

int main(void)
{
    led_init();
    fal_init();
    boot_check_key();

    phy_init();
    rt_hw_stm32_eth_init();
    telnet_module_init();
    finsh_system_init();
    
    dfs_mount(RT_NULL, "/", "rom", 0, &(romfs_root));
    internal_web_init();

    return RT_EOK;
}
