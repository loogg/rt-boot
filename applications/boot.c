#include "boot.h"
#include <finsh.h>
#include <fal.h>
#include "crc32.h"
#include "led.h"

#define DBG_TAG               "boot"
#define DBG_LVL               DBG_LOG
#include <rtdbg.h>

static const struct fal_partition *app_part = RT_NULL;
static const struct fal_partition *download_part = RT_NULL;

#define IAP_FIRM_BUF_SIZE 4096
static uint8_t firm_buf[IAP_FIRM_BUF_SIZE];

static int get_firm_header(const struct fal_partition *part, uint32_t off, firm_pkg_t *firm_pkg)
{
    if(fal_partition_read(part, off, (uint8_t *)firm_pkg, sizeof(firm_pkg_t)) < 0)
        return -RT_ERROR;
    return RT_EOK;
}

static int calc_part_firm_crc32(const struct fal_partition *part, uint32_t firm_len, uint32_t firm_off, uint32_t *calc_crc)
{
    int total_length = 0, length = 0;
    crc32_ctx ctx;
    if(((uint64_t)firm_len + firm_off) > part->len)
        return -RT_ERROR;

    crc32_init(&ctx);
    do
    {
        length = fal_partition_read(part, firm_off + total_length, firm_buf, firm_len - total_length > IAP_FIRM_BUF_SIZE ?
                                    IAP_FIRM_BUF_SIZE : firm_len - total_length);
        if(length <= 0)
            return -RT_ERROR;
        crc32_update(&ctx, firm_buf, length);
        total_length += length;

    } while (total_length < firm_len);
    
    crc32_final(&ctx, calc_crc);
    return RT_EOK;
}

static int check_part_firm(const struct fal_partition *part, firm_pkg_t *firm_pkg)
{
    uint32_t calc_crc, header_off, firm_off;
    header_off = 0;
    firm_off = sizeof(firm_pkg_t);
    if(rt_strcmp(part->name, "app") == 0)
    {
        header_off = part->len - sizeof(firm_pkg_t);
        firm_off = 0;
    }
    if (get_firm_header(part, header_off, firm_pkg) != RT_EOK)
        return -RT_ERROR;
    if(calc_part_firm_crc32(part, firm_pkg->raw_size, firm_off, &calc_crc) != RT_EOK)
        return -RT_ERROR;
    if(firm_pkg->body_crc32 != calc_crc)
    {
        LOG_E("Get firmware header occur CRC32(calc.crc: %08x != hdr.info_crc32: %08x) error on \'%s\' partition!",
              calc_crc, firm_pkg->body_crc32, part->name);
        return -RT_ERROR;
    }
    LOG_I("Verify \'%s\' partiton(fw ver: %s, timestamp: %d) success.", 
          part->name, firm_pkg->version_name, firm_pkg->time_stamp);
    return RT_EOK;
}

static void print_progress(size_t cur_size, size_t total_size)
{
    static uint8_t progress_sign[100 + 1];
    uint8_t i, per = cur_size * 100 / total_size;

    if (per > 100)
    {
        per = 100;
    }

    for (i = 0; i < 100; i++)
    {
        if (i < per)
        {
            progress_sign[i] = '=';
        }
        else if (per == i)
        {
            progress_sign[i] = '>';
        }
        else
        {
            progress_sign[i] = ' ';
        }
    }

    progress_sign[sizeof(progress_sign) - 1] = '\0';

    LOG_I("\033[2A");
    LOG_I("OTA Write: [%s] %d%%", progress_sign, per);
}

static int firm_upgrade(const struct fal_partition *src_part, firm_pkg_t *src_header)
{
    rt_err_t result = RT_EOK;
    firm_pkg_t app_header;
    uint32_t total_length = 0, length = 0;
    result = check_part_firm(app_part, &app_header);
    if((result == RT_EOK) && (rt_strcmp(src_header->version_name, app_header.version_name) == 0))
        return RT_EOK;
    if (src_header->raw_size + sizeof(firm_pkg_t) > app_part->len)
    {
        LOG_E("The partition \'app\' length is (%d), need (%d)!", app_part->len, 
              src_header->raw_size + sizeof(firm_pkg_t));
        return -RT_EFULL;
    }
    if(result == RT_EOK)
        LOG_I("OTA firmware(app) upgrade(%s->%s) startup.", app_header.version_name, src_header->version_name);
    else
        LOG_I("OTA firmware(app) upgrade startup.");
    LOG_I("The partition \'app\' is erasing.");

    rt_base_t level = rt_hw_interrupt_disable();
    result = fal_partition_erase_all(app_part);
    rt_hw_interrupt_enable(level);

    if(result < 0)
        return -RT_ERROR;
    LOG_I("The partition \'app\' erase success.");

    do
    {
        length = fal_partition_read(src_part, sizeof(firm_pkg_t) + total_length, firm_buf, src_header->raw_size - total_length > IAP_FIRM_BUF_SIZE ?
                                    IAP_FIRM_BUF_SIZE : src_header->raw_size - total_length);
        if(length <= 0)
            return -RT_ERROR;

        level = rt_hw_interrupt_disable();
        result = fal_partition_write(app_part, total_length, firm_buf, length);
        rt_hw_interrupt_enable(level);

        if (result <= 0)
            return -RT_ERROR;

        total_length += length;
        print_progress(total_length, src_header->raw_size);
        led_boot_mode();
    }while(total_length < src_header->raw_size);
    
    if(fal_partition_write(app_part, app_part->len - sizeof(firm_pkg_t), (uint8_t *)src_header, sizeof(firm_pkg_t)) < 0)
        return -RT_ERROR;

    result = check_part_firm(app_part, &app_header);
    return result;
}

int boot_act(void)
{
    app_part = fal_partition_find("app");
    if(app_part == RT_NULL)
    {
        LOG_E("Partition app not find.");
        return -RT_ERROR;
    }
    download_part = fal_partition_find("download");
    if(download_part == RT_NULL)
    {
        LOG_E("Partition download not find.");
        return -RT_ERROR;
    }

    rt_err_t result = RT_EOK;
    firm_pkg_t header;

    result = check_part_firm(download_part, &header);
    if(result != RT_EOK)
    {
        result = RT_EOK;
        LOG_E("Get OTA \"%s\" partition firmware header filed!", download_part->name);
        goto __exit;
    }
    
    result = firm_upgrade(download_part, &header);
    if((result == RT_EOK) || (result == -RT_EFULL))
    {
        result = RT_EOK;
        fal_partition_erase_all(download_part);
    }

__exit:
    led_boot_ready_mode();
    return result;
}
MSH_CMD_EXPORT(boot_act, boot act);
