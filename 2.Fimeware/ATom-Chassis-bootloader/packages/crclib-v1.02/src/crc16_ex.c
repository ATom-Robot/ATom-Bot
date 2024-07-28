/*
 * crc16_ex.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-03-12     qiyongzhong       first version
 */
    
#include "crc16_ex.h"

#ifdef CRCLIB_USING_CRC16_EX

void crc16_ex_init(crc16_inst_t *hinst, u16 poly)
{
    u32 i, j;
    u16 c;

    for (i = 0; i < 256; i++)
    {
        c = i;

        for (j = 0; j < 8; j++)
        {
            if(c & 0x0001)
                c = poly ^ (c >> 1);
            else
                c >>= 1;
        }

        hinst->table[i] = c;
    }
}

u16 crc16_ex_cyc_cal(crc16_inst_t *hinst, u16 init_val, u8 *pdata, u32 len)
{
    register u32 i;
    register u16 crc16;
    register u8 idx;

    crc16 = init_val;
    for(i=0; i<len; i++)
    {
        idx = ((u8)crc16) ^ (*pdata++);
        crc16 = (crc16>>8) ^ hinst->table[idx];
    }

    return(crc16);
}

u16 crc16_ex_cal(crc16_inst_t *hinst, u8 *pdata, u32 len)
{
    return(crc16_ex_cyc_cal(hinst, CRC16_EX_INIT_VAL, pdata, len) ^ CRC16_EX_INIT_VAL);
}
#endif

