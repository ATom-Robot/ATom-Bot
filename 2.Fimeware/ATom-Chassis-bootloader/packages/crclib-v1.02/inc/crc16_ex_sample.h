/*
 * crc16_ex_sample.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-03-15     qiyongzhong       first version
 */

#ifndef __CRC16_EX_SAMPLE_H__
#define __CRC16_EX_SAMPLE_H__

#include <crc.h>

#ifdef CRCLIB_USING_CRC16_EX
#ifdef CRCLIB_USING_CRC16_EX_SAMPLE

#define CRC16_EX_SAMPLE_POLY        0xA001

extern crc16_inst_t crc16_ex_sample_inst;

#define CRC16_EX_SAMPLE_CYC_CAL(init, pd, len)  crc16_ex_cyc_cal(&crc16_ex_sample_inst, init, pd, len)
#define CRC16_EX_SAMPLE_CAL(pd, len)            crc16_ex_cal(&crc16_ex_sample_inst, pd, len)

#endif
#endif
#endif

