/*
 * crc32_ex.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-03-12     qiyongzhong       first version
 */

#ifndef __CRC32_EX_H__
#define __CRC32_EX_H__

#include "typedef.h"
#include "crc_cfg.h"

#ifdef CRCLIB_USING_CRC32_EX

#define CRC32_EX_INIT_VAL   0xFFFFFFFF

typedef struct{
    u32 table[256];
}crc32_inst_t;

/* 
 * @brief   initialize crc instance
 * @param   hinst       - instance handle
 * @param   poly        - polynomial of crc
 * @retval  none
 */
void crc32_ex_init(crc32_inst_t *hinst, u32 poly);

/* 
 * @brief   cyclic calculation crc check value
 * @param   hinst       - instance handle
 * @param   init_val    - initial value
 * @param   pdata       - datas pointer
 * @param   len         - datas len
 * @retval  calculated result 
 */
u32 crc32_ex_cyc_cal(crc32_inst_t *hinst, u32 init_val, u8 *pdata, u32 len);

/* 
 * @brief   calculation crc check value, initial is 0xFFFFFFFF
 * @param   hinst       - instance handle
 * @param   pdata       - datas pointer
 * @param   len         - datas len
 * @retval  calculated result 
 */
u32 crc32_ex_cal(crc32_inst_t *hinst, u8 *pdata, u32 len);

#endif
#endif

