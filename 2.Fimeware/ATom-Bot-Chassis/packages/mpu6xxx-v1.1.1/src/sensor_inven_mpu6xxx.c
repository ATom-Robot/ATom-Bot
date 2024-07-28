/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-08     flybreak     the first version
 */

#include "sensor_inven_mpu6xxx.h"

#define DBG_TAG "sensor.inven.mpu6xxx"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define mpu_dev ((struct mpu6xxx_device *)sensor->parent.user_data)

static struct mpu6xxx_device *_mpu6xxx_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t  i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    return mpu6xxx_init(intf->dev_name, i2c_addr);
}

static rt_err_t _mpu6xxx_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        uint8_t range_ctr;

        if (range < 2000)
            range_ctr = MPU6XXX_ACCEL_RANGE_2G;
        else if (range < 4000)
            range_ctr = MPU6XXX_ACCEL_RANGE_4G;
        else if (range < 8000)
            range_ctr = MPU6XXX_ACCEL_RANGE_8G;
        else
            range_ctr = MPU6XXX_ACCEL_RANGE_16G;

        LOG_D("acce set range %d", range_ctr);

        return mpu6xxx_set_param(mpu_dev, MPU6XXX_ACCEL_RANGE, range_ctr);
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        uint8_t range_ctr;

        if (range < 250000UL)
            range_ctr = MPU6XXX_GYRO_RANGE_250DPS;
        else if (range < 500000UL)
            range_ctr = MPU6XXX_GYRO_RANGE_500DPS;
        else if (range < 1000000UL)
            range_ctr = MPU6XXX_GYRO_RANGE_1000DPS;
        else
            range_ctr = MPU6XXX_GYRO_RANGE_2000DPS;

        LOG_D("gyro set range %d", range);

        return mpu6xxx_set_param(mpu_dev, MPU6XXX_GYRO_RANGE, range_ctr);
    }
    return RT_EOK;
}

static rt_err_t _mpu6xxx_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _mpu6xxx_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    static uint8_t ref_count = 0;

    if (power == RT_SENSOR_POWER_DOWN)
    {
        if (ref_count > 0)
        {
            ref_count --;
        }
        if (ref_count == 0)
        {
            LOG_D("set power down");
            return mpu6xxx_set_param(mpu_dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_ENABLE);
        }
        return RT_EOK;
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        ref_count ++;
        LOG_D("set power normal");
        return mpu6xxx_set_param(mpu_dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
}

static rt_size_t _mpu6xxx_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        struct mpu6xxx_3axes acce;
        if (mpu6xxx_get_accel(mpu_dev, &acce) != RT_EOK)
        {
            return 0;
        }

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = acce.x;
        data->data.acce.y = acce.y;
        data->data.acce.z = acce.z;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        struct mpu6xxx_3axes gyro;
        if (mpu6xxx_get_gyro(mpu_dev, &gyro) != RT_EOK)
        {
            return 0;
        }

        data->type = RT_SENSOR_CLASS_GYRO;
        data->data.gyro.x = gyro.x * 100;
        data->data.gyro.y = gyro.y * 100;
        data->data.gyro.z = gyro.z * 100;
        data->timestamp = rt_sensor_get_ts();
    }
#ifdef PKG_USING_MPU6XXX_MAG
    else if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        struct mpu6xxx_3axes mag;
        if (mpu6xxx_get_mag(mpu_dev, &mag) != RT_EOK)
        {
            return 0;
        }

        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = mag.x * 10;
        data->data.mag.y = mag.y * 10;
        data->data.mag.z = mag.z * 10;
        data->timestamp = rt_sensor_get_ts();
    }
#endif
    return 1;
}

static rt_size_t mpu6xxx_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mpu6xxx_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mpu6xxx_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = mpu_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mpu6xxx_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _mpu6xxx_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _mpu6xxx_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mpu6xxx_fetch_data,
    mpu6xxx_control
};

int rt_hw_mpu6xxx_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mpu6xxx_device *mpu_dev_temp;
    rt_sensor_t sensor_acce = RT_NULL, sensor_gyro = RT_NULL, sensor_mag = RT_NULL;

    mpu_dev_temp = _mpu6xxx_init(&cfg->intf);
    if (mpu_dev_temp == RT_NULL)
    {
        LOG_E("_mpu6xxx init err!");
        goto __exit;
    }

#ifdef PKG_USING_MPU6XXX_ACCE
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            goto __exit;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_acce->info.model      = "mpu6xxx_acc";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = 16000;
        sensor_acce->info.range_min  = 2000;
        sensor_acce->info.period_min = 5;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, mpu_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto acce__exit;
        }
    }
#endif
#ifdef PKG_USING_MPU6XXX_GYRO
    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto gyro__exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_gyro->info.model      = "mpu6xxx_gyro";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MDPS;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_gyro->info.range_max  = 2000000;
        sensor_gyro->info.range_min  = 250000;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, mpu_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto gyro__exit;
        }
    }
#endif

#ifdef PKG_USING_MPU6XXX_MAG
    /* magnetometer sensor register */
    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mag == RT_NULL)
            goto mag__exit;

        sensor_mag->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mag->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_mag->info.model      = "mpu6xxx_mag";
        sensor_mag->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_mag->info.intf_type  = RT_SENSOR_INTF_I2C | RT_SENSOR_INTF_SPI;
        sensor_mag->info.range_max  = 49120;
        sensor_mag->info.range_min  = -49120;
        sensor_mag->info.period_min = 100;

        rt_memcpy(&sensor_mag->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mag->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, mpu_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto mag__exit;
        }
    }
#endif

    LOG_I("sensor init success");
    return RT_EOK;

#ifdef PKG_USING_MPU6XXX_MAG
mag__exit:
    if (sensor_mag)
        rt_free(sensor_mag);
#endif

#ifdef PKG_USING_MPU6XXX_GYRO
    rt_device_unregister((rt_device_t)sensor_gyro);
gyro__exit:
    if (sensor_gyro)
        rt_free(sensor_gyro);
#endif

#ifdef PKG_USING_MPU6XXX_ACCE
    rt_device_unregister((rt_device_t)sensor_acce);
acce__exit:
    if (sensor_acce)
        rt_free(sensor_acce);
#endif

__exit:
    if (mpu_dev_temp)
        mpu6xxx_deinit(mpu_dev_temp);

    return -RT_ERROR;
}
