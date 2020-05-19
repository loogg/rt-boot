#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <dfs_fs.h>
#include "dfs_romfs.h"
#include <fal.h>

int main(void)
{
    dfs_mount(RT_NULL, "/", "rom", 0, &(romfs_root));
    fal_init();
    
    static RTC_HandleTypeDef RTC_Handler;
    RTC_Handler.Instance = RTC; 
    
    HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR0, 0xA5A5);

    return RT_EOK;
}
