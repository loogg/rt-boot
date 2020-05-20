#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <dfs_fs.h>
#include "dfs_romfs.h"
#include <fal.h>
#include "internal_web.h"

#define BUTTON_PIN      GET_PIN(E, 1)

int main(void)
{
    dfs_mount(RT_NULL, "/", "rom", 0, &(romfs_root));
    fal_init();
    
    internal_web_init();
    // RTC_HandleTypeDef RTC_Handler;
    // RTC_Handler.Instance = RTC; 
    
    // HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR0, 0xA5A5);

    return RT_EOK;
}
