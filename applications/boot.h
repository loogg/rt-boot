#ifndef __BOOT_H
#define __BOOT_H
#include <board.h>


typedef struct
{
    char head[8];
    uint32_t time_stamp;
    char app_part_name[16];
    char version_name[24];
    char res[24];
    uint32_t body_crc32;
    uint32_t hash_code;
    uint32_t raw_size;
    uint32_t pkg_size;
    uint32_t hdr_crc32;
}firm_pkg_t;

int boot_act(void);

#endif
