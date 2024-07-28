#ifndef _I2C_UTILS_H_
#define _I2C_UTILS_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>

#ifdef I2C_TOOLS_USE_SW_I2C
    #include "SoftwareI2C.h"
#endif

#define I2C_TOOLS_BUFFER_SIZE 64

#define ARG_CMD_POS             1

#ifdef I2C_TOOLS_USE_SW_I2C
    #define ARG_SDA_POS         2
    #define ARG_SCL_POS         3
    #define ARG_ADDR_POS        4
    #define ARG_DATA_POS        5
    #define ARG_READ_MAX        7
#else
    #define ARG_BUS_NAME_POS    2
    #define ARG_ADDR_POS        3
    #define ARG_DATA_POS        4
    #define ARG_READ_MAX        6
#endif

long strtonum(const char* str);

void i2c_help();

#ifdef I2C_TOOLS_USE_SW_I2C
    rt_bool_t i2c_init(int sda, int scl);
#else
    rt_bool_t i2c_init(const char* busname);
#endif

int  i2c_probe(char addr);
void i2c_scan();

rt_bool_t  i2c_write(char addr, rt_uint8_t* data, int len);
rt_uint8_t i2c_read(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t* buffer, rt_uint8_t len);

#endif /*_I2C_UTILS_H_*/
