#include "led.h"
#include <board.h>
#include <agile_led.h>

//定义信号灯引脚
#define LED1_PIN GET_PIN(E, 6)
#define LED2_PIN GET_PIN(E, 5)
#define LED3_PIN GET_PIN(E, 4)
#define LED4_PIN GET_PIN(E, 3)
#define LED5_PIN GET_PIN(E, 2)

//定义信号灯对象
static agile_led_t *led1 = RT_NULL;
static agile_led_t *led2 = RT_NULL;
static agile_led_t *led3 = RT_NULL;
static agile_led_t *led4 = RT_NULL;
static agile_led_t *led5 = RT_NULL;

static uint8_t boot_ready_flag = 0;
static uint8_t recovery_flag = 0;

static const char *boot_ready_light_mode = "200,200";
static const char *recovery_light_mode = "1000,1000";

void led_boot_ready_mode(void)
{
    if(boot_ready_flag)
        return;
    
    boot_ready_flag = 1;
    agile_led_stop(led1);
    agile_led_set_light_mode(led1, boot_ready_light_mode, -1);
    agile_led_start(led1);
}

void led_recovery_mode(void)
{
    if(recovery_flag)
        return;
    
    recovery_flag = 1;
    agile_led_stop(led1);
    agile_led_set_light_mode(led1, recovery_light_mode, -1);
    agile_led_start(led1);
}

void led_boot_mode(void)
{
    if(boot_ready_flag)
    {
        boot_ready_flag = 0;
        agile_led_stop(led1);
    }
    agile_led_toggle(led1);
}

int led_init(void)
{
    //初始化信号灯对象
    led1 = agile_led_create(LED1_PIN, PIN_HIGH, RT_NULL, 0);
    led2 = agile_led_create(LED2_PIN, PIN_HIGH, RT_NULL, 0);
    led3 = agile_led_create(LED3_PIN, PIN_HIGH, RT_NULL, 0);
    led4 = agile_led_create(LED4_PIN, PIN_HIGH, RT_NULL, 0);
    led5 = agile_led_create(LED5_PIN, PIN_HIGH, RT_NULL, 0);

    RT_ASSERT(led1 != RT_NULL);
    RT_ASSERT(led2 != RT_NULL);
    RT_ASSERT(led3 != RT_NULL);
    RT_ASSERT(led4 != RT_NULL);
    RT_ASSERT(led5 != RT_NULL);

    agile_led_on(led1);
    agile_led_on(led2);
    agile_led_on(led3);
    agile_led_on(led4);
    agile_led_on(led5);
    rt_thread_mdelay(200);
    agile_led_off(led1);
    agile_led_off(led2);
    agile_led_off(led3);
    agile_led_off(led4);
    agile_led_off(led5);

    return RT_EOK;
}
