/*
 * crc16_ex_sample.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-03-15     qiyongzhong       first version
 */

#include <rtthread.h>
#include <crc16_ex_sample.h>

#ifdef CRCLIB_USING_CRC16_EX
#ifdef CRCLIB_USING_CRC16_EX_SAMPLE

crc16_inst_t crc16_ex_sample_inst;

static int crc16_ex_sample_init(void)
{
    crc16_ex_init(&crc16_ex_sample_inst, CRC16_EX_SAMPLE_POLY);
    return(0);
}
INIT_BOARD_EXPORT(crc16_ex_sample_init);

#ifdef CRCLIB_USING_CRC16
#if (CRC16_EX_SAMPLE_POLY == CRC16_POLY)
static void crc16_ex_test(void)
{
    u8 data[128];

    for (int i=0; i<sizeof(data); i++)
    {
        data[i] = 0x10 + i;
    }

    u16 crc16_rst = crc16_cal(data, sizeof(data));
    rt_kprintf("crc16_cal result = 0x%04X\n", crc16_rst);
    
    u16 crc16_ex_rst = CRC16_EX_SAMPLE_CAL(data, sizeof(data));
    rt_kprintf("crc16_ex_cal result = 0x%04X\n", crc16_ex_rst);

    if (crc16_rst == crc16_ex_rst)
    {
        rt_kprintf("crc16_ex_test is pass.\n");
    }   
}
MSH_CMD_EXPORT(crc16_ex_test, crc16_ex functions test);
#endif
#endif

#endif
#endif

