/*
 * crc32_ex.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-03-12     qiyongzhong       first version
 */
    
#include "crc32_ex.h"

#ifdef CRCLIB_USING_CRC32_EX

void crc32_ex_init(crc32_inst_t *hinst, u32 poly)
{
    u32 i, j;
    u32 c;

    for (i = 0; i < 256; i++)
    {
        c = i;

        for (j = 0; j < 8; j++)
        {
            if(c & 0x00000001)
                c = poly ^ (c >> 1);
            else
                c >>= 1;
        }

        hinst->table[i] = c;
    }
}

u32 crc32_ex_cyc_cal(crc32_inst_t *hinst, u32 init_val, u8 *pdata, u32 len)
{
    register u32 i;
    register u32 crc32;
    register u8 idx;

    crc32 = init_val;
    for(i=0; i<len; i++)
    {
        idx = ((u8)crc32) ^ (*pdata++);
        crc32 = (crc32>>8) ^ hinst->table[idx];
    }

    return(crc32);
}

u32 crc32_ex_cal(crc32_inst_t *hinst, u8 *pdata, u32 len)
{
    return(crc32_ex_cyc_cal(hinst, CRC32_EX_INIT_VAL, pdata, len) ^ CRC32_EX_INIT_VAL);
}
#endif

