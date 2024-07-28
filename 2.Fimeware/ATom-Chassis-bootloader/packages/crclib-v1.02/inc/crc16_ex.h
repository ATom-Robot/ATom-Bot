/*
 * crc16_ex.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-03-12     qiyongzhong       first version
 */

#ifndef __CRC16_EX_H__
#define __CRC16_EX_H__

#include "typedef.h"
#include "crc_cfg.h"

#ifdef CRCLIB_USING_CRC16_EX

#define CRC16_EX_INIT_VAL   0xFFFF

typedef struct{
    u16 table[256];
}crc16_inst_t;

/* 
 * @brief   initialize crc instance
 * @param   hinst       - instance handle
 * @param   poly        - polynomial of crc
 * @retval  none
 */
void crc16_ex_init(crc16_inst_t *hinst, u16 poly);

/* 
 * @brief   cyclic calculation crc check value
 * @param   hinst       - instance handle
 * @param   init_val    - initial value
 * @param   pdata       - datas pointer
 * @param   len         - datas len
 * @retval  calculated result 
 */
u16 crc16_ex_cyc_cal(crc16_inst_t *hinst, u16 init_val, u8 *pdata, u32 len);

/* 
 * @brief   calculation crc check value, initial is 0xFFFF
 * @param   hinst       - instance handle
 * @param   pdata       - datas pointer
 * @param   len         - datas len
 * @retval  calculated result 
 */
u16 crc16_ex_cal(crc16_inst_t *hinst, u8 *pdata, u32 len);

#endif
#endif

