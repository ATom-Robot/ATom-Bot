
/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-vl53l0x>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-16     panrui      the first version
 */

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "vl53l0x_platform.h"

//#ifdef PKG_USING_VL53L0X

#define DBG_TAG "vl53l0x"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* vl53l0x i2c slave address */
#define VL53L0X_ADDR (0x52)	

/* vl53l0x message max size */
#define VL53L0X_MAX_MESSAGE_LEN 	(64) 

/* delay function */
#define VL53L0X_OsDelay(...) rt_thread_delay(1) 

/* when not customized by application define dummy one */
#ifndef VL53L0X_GetI2cBus

/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_GetI2cBus(...) (void)0
#endif

#ifndef VL53L0X_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_PutI2cBus(...) (void)0
#endif

#ifndef VL53L0X_OsDelay
#   define  VL53L0X_OsDelay(...) (void)0
#endif

/* vl53l0x for RT-Thread sensor device */
#define	VL53L0X_DIST_RANGE_MAX	(200)	/* 1mm */	
#define	VL53L0X_DIST_RANGE_MIN	(0)		/* 1mm */
#define VL53L0X_DIST_PEROID     (100)	/* 10ms */


static VL53L0X_Error Vl53l0xWriteRegs(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint8_t data_size)
{
	return dev->RegWrite(dev->I2cDevAddr, reg, data, data_size);
}

static VL53L0X_Error Vl53l0xReadRegs(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint8_t data_size)
{
	return dev->RegRead(dev->I2cDevAddr, reg, data, data_size);
}

/* the ranging_sensor_comms.dll will take care of the page selection */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
	int32_t status_int;
	
    if (count > VL53L0X_MAX_MESSAGE_LEN - 1) 
	{
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
	status_int = Vl53l0xWriteRegs(dev, index, pdata, count);
    VL53L0X_GetI2cBus();
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

/* the ranging_sensor_comms.dll will take care of the page selection */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	
    VL53L0X_GetI2cBus();
	status_int = Vl53l0xReadRegs(dev, index, pdata, count);
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
	status_int = Vl53l0xWriteRegs(dev, index, &data, 1);
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t buf[2] = {0};
	
    buf[0] = data >> 8;
    buf[1] = data & 0x00FF;

    VL53L0X_GetI2cBus();
	status_int = Vl53l0xWriteRegs(dev, index, &buf[0], 2);
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t buf[4] = {0};
  
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8)  & 0xFF;
    buf[3] = (data >> 0 ) & 0xFF;
    VL53L0X_GetI2cBus();
	status_int = Vl53l0xWriteRegs(dev, index, &buf[0], 4);
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t and_data, uint8_t or_data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t data;

    status = VL53L0X_RdByte(dev, index, &data);
    if (status)
	{
        goto done;
    }
    data = (data & and_data) | or_data;
    status = VL53L0X_WrByte(dev, index, data);
done:
    return status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = Vl53l0xReadRegs(dev, index, data, 1);
    if(status_int != 0)
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t buf[2] = {0};
	
    VL53L0X_GetI2cBus();
    status_int = Vl53l0xReadRegs(dev, index, &buf[0], 2);

    if(status_int != 0)
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)buf[0]<<8) + (uint16_t)buf[1];
done:
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t buf[4] = {0};
	
    VL53L0X_GetI2cBus();
    status_int = Vl53l0xReadRegs(dev, index, &buf[0], 4);
    if (status_int != 0) 
	{
        status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)buf[0]<<24) + ((uint32_t)buf[1]<<16) + ((uint32_t)buf[2]<<8) + (uint32_t)buf[3];

done:
    VL53L0X_PutI2cBus();
    return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
	
    VL53L0X_OsDelay();
	
    return status;
}
