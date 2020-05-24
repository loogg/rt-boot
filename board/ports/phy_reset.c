/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-23     flybreak     first version
 */

#include <board.h>

#define RESET_IO GET_PIN(C, 2)

void phy_reset(void)
{
    rt_pin_write(RESET_IO, PIN_HIGH);
    rt_thread_mdelay(50);
    rt_pin_write(RESET_IO, PIN_LOW);
}

int phy_init(void)
{
    rt_pin_mode(RESET_IO, PIN_MODE_OUTPUT);
    rt_pin_write(RESET_IO, PIN_LOW);
    return RT_EOK;
}
