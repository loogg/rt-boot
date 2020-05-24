#include <webnet.h>
#include <wn_module.h>
#include <string.h>
#include <stdio.h>
#include <fal.h>

static int file_size = 0;
static uint8_t update_ok = 1;

static const char *get_file_name(struct webnet_session *session)
{
    const char *path = RT_NULL, *path_last = RT_NULL;
    
    path_last = webnet_upload_get_filename(session);
    if (path_last == RT_NULL)
    {
        rt_kprintf("file name err!!\n");
        return RT_NULL;
    }

    path = strrchr(path_last, '\\');
    if (path != RT_NULL)
    {
        path++;
        path_last = path;
    }

    path = strrchr(path_last, '/');
    if (path != RT_NULL)
    {
        path++;
        path_last = path;
    }
    
    return path_last;
}

static int upload_open(struct webnet_session *session)
{
    const char *file_name = RT_NULL;
    file_size = 0;
    update_ok = 1;

    file_name = get_file_name(session);
    rt_kprintf("Upload FileName: %s\n", file_name);
    rt_kprintf("Content-Type   : %s\n", webnet_upload_get_content_type(session));

    const struct fal_partition *part = fal_partition_find("app");
    if(part == RT_NULL)
        return RT_NULL;
    
    int len = fal_partition_erase_all(part);
    if(len != part->len)
        return RT_NULL;
    
    return (int)part;
}

static int upload_close(struct webnet_session* session)
{
    return 0;
}

static int upload_write(struct webnet_session* session, const void* data, rt_size_t length)
{
    if(update_ok == 0)
        return 0;
    
    const struct fal_partition *part = (const struct fal_partition *)webnet_upload_get_userdata(session);
    if(part == RT_NULL)
    {
        rt_kprintf("partition app NULL\r\n");
        update_ok = 0;
        return 0;
    }

    if(file_size + length > part->len)
    {
        rt_kprintf("file size is too large\r\n");
        update_ok = 0;
        return 0;
    }
    
    int len = fal_partition_write(part, file_size, data, length);
    if(len != length)
    {
        rt_kprintf("write error\r\n");
        update_ok = 0;
        return 0;
    }

    uint8_t *read_buf = (uint8_t *)rt_malloc(length);
    len = fal_partition_read(part, file_size, read_buf, length);
    if(len != length)
    {
        rt_free(read_buf);
        read_buf = RT_NULL;
        rt_kprintf("read error\r\n");
        update_ok = 0;
        return 0;
    }

    if(memcmp(data, read_buf, length) != 0)
    {
        rt_free(read_buf);
        read_buf = RT_NULL;
        rt_kprintf("verify error\r\n");
        update_ok = 0;
        return 0;
    }
    
    rt_free(read_buf);
    read_buf = RT_NULL;
    
    file_size += length;

    return length;
}

static int upload_done (struct webnet_session* session)
{
    const char *mimetype;

    rt_kprintf("Upload done.\r\n");
    
    char tmp[100] = "";
    snprintf(tmp, sizeof(tmp), "{\"code\":%d,\"filesize\":%d}", update_ok ? 0 : -1, file_size);

    /* get mimetype */
    mimetype = mime_get_type(".html");

    /* set http header */
    session->request->result_code = 200;
    webnet_session_set_header(session, mimetype, 200, "Ok", rt_strlen(tmp));
    webnet_session_printf(session, tmp);

    return 0;
}

const struct webnet_module_upload_entry upload_entry_app =
{
    "/app",
    upload_open,
    upload_close,
    upload_write,
    upload_done
};
