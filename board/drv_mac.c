#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#define DRV_DEBUG
#define LOG_TAG             "drv.mac"
#include <drv_log.h>

#define MAC_DEVICE_ADDR     0x50
#define MAC_BUS_DEVICE_NAME "i2c2"

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

static struct rt_device mac;

static rt_err_t read_regs(struct rt_i2c_bus_device *bus, uint8_t reg, uint8_t len, uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];
    msgs[0].addr = MAC_DEVICE_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = MAC_DEVICE_ADDR; 
    msgs[1].flags = RT_I2C_RD; 
    msgs[1].buf = buf; 
    msgs[1].len = len;

    if (rt_i2c_transfer(bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static void rt_mac_init(void)
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(MAC_BUS_DEVICE_NAME);
}

#define NIOCTL_GADDR		0x01
static rt_err_t rt_mac_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = -RT_ERROR;

    RT_ASSERT(dev != RT_NULL);
    switch(cmd)
    {
        case NIOCTL_GADDR:
            result = read_regs(i2c_bus, 0xFA, 6, (uint8_t *)args);
            if(result != RT_EOK)
                LOG_E("read mac error.");
            else
            {
                uint8_t *ptr = (uint8_t *)args;
                LOG_D("read mac: %X %X %X %X %X %X", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
            }
        break;

        default:
        break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops mac_ops = 
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_mac_control
};
#endif

static rt_err_t rt_hw_mac_register(rt_device_t device, const char *name, rt_uint32_t flag)
{
    RT_ASSERT(device != RT_NULL);

    rt_mac_init();
#ifdef RT_USING_DEVICE_OPS
    device->ops         = &mac_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_mac_control;
#endif
    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

int rt_hw_mac_init(void)
{
    rt_err_t result;
    result = rt_hw_mac_register(&mac, "mac", RT_DEVICE_FLAG_RDWR);
    if(result != RT_EOK)
    {
        LOG_E("mac register err code: %d", result);
        return result;
    }
    LOG_D("mac init success");
    return RT_EOK;
}
INIT_PREV_EXPORT(rt_hw_mac_init);
