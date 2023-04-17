
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
#include "sensor.h"
#include "vl53l0x.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

#ifdef PKG_USING_VL53L0X

#define DBG_TAG "vl53l0x"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* vl53l0x for RT-Thread */
#define	VL53L0X_I2C_BUS			"i2c1"		/* i2c linked */
#define VL53L0X_DEVICE_NAME		"vl53l0x"	/* register device name */

/* vl53l0x for RT-Thread sensor device */
#define	VL53L0X_DIST_RANGE_MAX	(2000)	/* 1mm */	
#define	VL53L0X_DIST_RANGE_MIN	(0)		/* 1mm */
#define VL53L0X_DIST_PEROID     (100)	/* 1ms */

static struct rt_i2c_bus_device *i2c_bus_dev;/* i2c bus device */
static rt_base_t xshutdown_pin = 0;/* shutdown control pin */
static VL53L0X_Dev_t vl53l0x_dev = /* vl53l0x device */
{
	.comms_type = 1,
	.comms_speed_khz = 400,
};	

int32_t vl53l0x_write_regs(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint16_t data_size)
{
  	struct rt_i2c_msg msg[2]={0};
	
    msg[0].addr		= slave_addr;
    msg[0].flags	= RT_I2C_WR;
    msg[0].len   	= 1;
    msg[0].buf   	= &reg;
    msg[1].addr  	= slave_addr;
    msg[1].flags	= RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len   	= data_size;
    msg[1].buf   	= data;
    if(rt_i2c_transfer(i2c_bus_dev, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

int32_t vl53l0x_read_regs(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint16_t data_size)
{
  	struct rt_i2c_msg msg[2]={0};
	
    msg[0].addr  = slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg;
    msg[1].addr  = slave_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = data_size;
    msg[1].buf   = data;

    if(rt_i2c_transfer(i2c_bus_dev, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus read failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t vl53l0x_set_power(rt_sensor_t psensor, rt_uint8_t power)
{	
    if (power == RT_SENSOR_POWER_DOWN)
    {
	  	rt_pin_write(xshutdown_pin, PIN_LOW);
		psensor->config.power = RT_SENSOR_POWER_DOWN;
		return RT_EOK;
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
       	rt_pin_write(xshutdown_pin, PIN_HIGH); 
		psensor->config.power = RT_SENSOR_POWER_NORMAL;
		return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
	
}

VL53L0X_Error vl53l0x_single_ranging_mode(VL53L0X_Dev_t *pdev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(pdev); 
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefCalibration(pdev,
        		&VhvSettings, &PhaseCal); 
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefSpadManagement(pdev,
        		&refSpadCount, &isApertureSpads); 
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        /* no need to do this when we use VL53L0X_PerformSingleRangingMeasurement */
        Status = VL53L0X_SetDeviceMode(pdev, VL53L0X_DEVICEMODE_SINGLE_RANGING); 
    }

    /* Enable/Disable Sigma and Signal check */
    if (Status == VL53L0X_ERROR_NONE) 
	{
        Status = VL53L0X_SetLimitCheckEnable(pdev,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
	{
        Status = VL53L0X_SetLimitCheckEnable(pdev,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) 
	{
        Status = VL53L0X_SetLimitCheckEnable(pdev,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (Status == VL53L0X_ERROR_NONE)
	{
        Status = VL53L0X_SetLimitCheckValue(pdev,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
        		(FixPoint1616_t)(1.5*0.023*65536));
    }

    return Status;
}

static rt_err_t vl53l0x_single_read_data(VL53L0X_Dev_t *pdev, VL53L0X_RangingMeasurementData_t *data)
{
  	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	
	status = VL53L0X_PerformSingleRangingMeasurement(pdev, data);
	
	if (VL53L0X_ERROR_NONE != status)
	{
		return -RT_ERROR;
	}
	return RT_EOK;
}

static rt_size_t vl53l0x_polling_get_data(rt_sensor_t psensor, struct rt_sensor_data *sensor_data)
{
  	VL53L0X_Dev_t *pdev;
	static VL53L0X_RangingMeasurementData_t vl53l0x_data;
	
	pdev = (VL53L0X_Dev_t*)psensor->parent.user_data;
	if(psensor->info.type == RT_SENSOR_CLASS_TOF)
	{/* actual distance */
	  	if (vl53l0x_single_read_data(pdev, &vl53l0x_data) == RT_EOK)
		{
		  	sensor_data->type = RT_SENSOR_CLASS_TOF;
			sensor_data->data.proximity = vl53l0x_data.RangeMilliMeter;
			sensor_data->timestamp = rt_sensor_get_ts();
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	
    return 1;
}

static rt_size_t vl53l0x_fetch_data(struct rt_sensor_device *psensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);
	RT_ASSERT(psensor);
	
	/*if(psensor->parent.open_flag & RT_DEVICE_FLAG_RDONLY)*/
	if(psensor->config.mode == RT_SENSOR_MODE_POLLING)
	{
        return vl53l0x_polling_get_data(psensor, buf);
    }

    return 0;
}

static rt_err_t vl53l0x_control(struct rt_sensor_device *psensor, int cmd, void *args)
{
	rt_err_t	ret = RT_EOK;
	rt_uint8_t  *p = RT_NULL;
	VL53L0X_DeviceInfo_t vl53l0x_info;
	
    RT_ASSERT(psensor);
	RT_ASSERT(args != RT_NULL);
	
    switch (cmd)
    {
        case RT_SENSOR_CTRL_GET_ID:
		  	
			p = (rt_uint8_t*)args;
		  	if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&vl53l0x_dev, &vl53l0x_info))
			{
				*p = vl53l0x_info.ProductType;
			}
			else
			{
				*p = 0xff;
			}
        break;
		
		case RT_SENSOR_CTRL_GET_INFO:
		 	rt_memcpy((rt_uint8_t*)args, (const rt_uint8_t*)&(psensor->info), sizeof(psensor->info));
		break;
		
		case RT_SENSOR_CTRL_SET_POWER:
			vl53l0x_set_power(psensor, (rt_uint32_t)args & 0xff);
	  	break;
		
        default:
        break;
	}
    return ret;
}

static struct rt_sensor_ops vl53l0x_ops =
{
    vl53l0x_fetch_data,
    vl53l0x_control,
};

int rt_hw_vl53l0x_init(const char *name, struct rt_sensor_config *cfg, rt_base_t xsht_pin)
{
  	rt_err_t ret = RT_EOK;
	rt_sensor_t sensor_dist = RT_NULL;
	VL53L0X_DeviceInfo_t vl53l0x_info;
	struct rt_i2c_bus_device *i2c_bus = RT_NULL;	/* linked i2c bus */
	
	xshutdown_pin = xsht_pin;
	rt_pin_mode(xsht_pin, PIN_MODE_OUTPUT);
	rt_pin_write(xsht_pin, PIN_LOW);
	rt_thread_delay(10);
	rt_pin_write(xsht_pin, PIN_HIGH);
	
    i2c_bus = rt_i2c_bus_device_find(cfg->intf.dev_name);
    if(i2c_bus == RT_NULL)
    {
        LOG_E("i2c bus device %s not found!\r\n", cfg->intf.dev_name);
		ret = -RT_ERROR;
		goto __exit;
    }	
	i2c_bus_dev = i2c_bus;
	vl53l0x_dev.I2cDevAddr = (rt_uint32_t)(cfg->intf.user_data) & 0xff;
	vl53l0x_dev.RegRead = vl53l0x_read_regs;
	vl53l0x_dev.RegWrite = vl53l0x_write_regs;
	
    /* tof sensor register */
    {
        sensor_dist = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_dist == RT_NULL)
		{
			goto __exit;
		}
		rt_memset(sensor_dist, 0x0, sizeof(struct rt_sensor_device));
        sensor_dist->info.type       = RT_SENSOR_CLASS_TOF;
        sensor_dist->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_dist->info.model      = "vl53l0x";
        sensor_dist->info.unit       = RT_SENSOR_UNIT_MM;
        sensor_dist->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_dist->info.range_max  = VL53L0X_DIST_RANGE_MAX;	
        sensor_dist->info.range_min  = VL53L0X_DIST_RANGE_MIN;
        sensor_dist->info.period_min = VL53L0X_DIST_PEROID;	/* read ten times in 1 second */

        rt_memcpy(&sensor_dist->config, cfg, sizeof(struct rt_sensor_config));
        sensor_dist->ops = &vl53l0x_ops;
        
        ret = rt_hw_sensor_register(sensor_dist, name, RT_DEVICE_FLAG_RDWR, (void*)&vl53l0x_dev/* private data */);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
   
	/* vl53l0x init */
	if (VL53L0X_ERROR_NONE != VL53L0X_DataInit(&vl53l0x_dev))
	{
		LOG_E("vl53l0x data init failed\r\n");
		goto __exit;	
	} 
	
	/* vl53l0x read version */
	if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&vl53l0x_dev, &vl53l0x_info))
	{
		LOG_I("vl53l0x info:\n      Name[%s]\n      Type[%s]\n      ProductId[%s]\r\n",
			  vl53l0x_info.Name, vl53l0x_info.Type, vl53l0x_info.ProductId);
	}	
	
	/* set single ranging mode */
	if (VL53L0X_ERROR_NONE != vl53l0x_single_ranging_mode(&vl53l0x_dev))
	{
	  	LOG_E("vl53l0x single ranging init failed\r\n");
		goto __exit;	
	}

	return RT_EOK;
	
__exit:
	if(sensor_dist)
	{
		rt_free(sensor_dist);
	}
	
    return ret;
}
#endif