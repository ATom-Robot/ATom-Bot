/*
 * crc8_ex.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-03-12     qiyongzhong       first version
 */
    
#include "crc8_ex.h"

#ifdef CRCLIB_USING_CRC8_EX

void crc8_ex_init(crc8_inst_t *hinst, u8 poly)
{
    u32 i, j;
    u8 c;

    for (i = 0; i < 256; i++)
    {
        c = i;

        for (j = 0; j < 8; j++)
        {
            if(c & 0x01)
                c = poly ^ (c >> 1);
            else
                c >>= 1;
        }

        hinst->table[i] = c;
    }
}

u8 crc8_ex_cyc_cal(crc8_inst_t *hinst, u8 init_val, u8 *pdata, u32 len)
{
    register u32 i;
    register u8 crc8;
    register u8 idx;

    crc8 = init_val;
    for(i=0; i<len; i++)
    {
        idx = ((u8)crc8) ^ (*pdata++);
        crc8 = (crc8>>8) ^ hinst->table[idx];
    }

    return(crc8);
}

u8 crc8_ex_cal(crc8_inst_t *hinst, u8 *pdata, u32 len)
{
    return(crc8_ex_cyc_cal(hinst, CRC8_EX_INIT_VAL, pdata, len) ^ CRC8_EX_INIT_VAL);
}
#endif

