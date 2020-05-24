#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define WDT_DEVICE_NAME    "wdt"   
static rt_device_t device = RT_NULL;  

void dog_feed(void *parameter)
{
    while(1)
    {
        rt_device_control(device, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
        rt_thread_mdelay(1000);
    }
}

static int wdog_init(void)
{
    device = rt_device_find(WDT_DEVICE_NAME);
    if(device == RT_NULL)
        HAL_NVIC_SystemReset();

    rt_err_t rc = rt_device_init(device);
    if(rc != RT_EOK)
        HAL_NVIC_SystemReset();
    
    rc = rt_device_control(device, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if(rc != RT_EOK)
        HAL_NVIC_SystemReset();
    
    rt_thread_t tid = rt_thread_create("wdog", dog_feed, RT_NULL, 512, RT_THREAD_PRIORITY_MAX - 2, 10);
    RT_ASSERT(tid != RT_NULL);
    rt_thread_startup(tid);

    return RT_EOK;
}
INIT_PREV_EXPORT(wdog_init);
