/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 * 2021-09-09     scratch-er   added setting and getting sensor offsets
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "mpu6xxx"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "mpu6xxx.h"
#include "mpu6xxx_reg.h"

#ifdef PKG_USING_MPU6XXX_MAG
#include "ak8963_reg.h"
#endif

#define MPU6XXX_ACCEL_SEN     (16384)
#define MPU6XXX_GYRO_SEN      (1310)

#define MPU60X0_SPI_MAX_SPEED (1000 * 1000)
#define MPU60X0_TEMP_SEN      (340)
#define MPU60X0_TEMP_OFFSET   (36.5)

#define MPU6500_TEMP_SEN      (333.87)
#define MPU6500_TEMP_OFFSET   (21)

// MAG
#define AK8963_RANGE          (4912)
#define AK8963_FULLSCALE      (32760)

/**
 * This function writes the value of the register for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t mpu6xxx_write_reg(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = 2;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }
    return res;
}

/**
 * This function reads the value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t mpu6xxx_read_regs(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf, len);
#endif
    }
    return res;
}

/**
 * This function writes a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t data)
{
    rt_uint8_t byte;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    byte = (data != 0) ? (byte | (1 << bit)) : (byte & ~(1 << bit));

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t *data)
{
    rt_uint8_t byte;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    *data = byte & (1 << bit);

    return RT_EOK;
}

/**
 * This function writes multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t data)
{
    rt_uint8_t byte, mask;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    data <<= (start_bit - len + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t *data)
{
    rt_uint8_t byte, mask;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    byte &= mask;
    byte >>= (start_bit - len + 1);
    *data = byte;

    return RT_EOK;
}

// MAG
#ifdef PKG_USING_MPU6XXX_MAG

#define MAG_READ_DELAY_TIME         50

static void mpu92_mag_write_reg(struct mpu6xxx_device *dev, rt_uint8_t addr, rt_uint8_t data)
{
    rt_uint8_t  status = 0;
    rt_uint32_t timeout = MAG_READ_DELAY_TIME;

    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_REG, addr);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_DO, data);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);

    do
    {
        mpu6xxx_read_regs(dev, MPU6XXX_RA_I2C_MST_STATUS, 1, &status);
        rt_thread_mdelay(1);
    } while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

}

#endif // PKG_USING_MPU6XXX_MAG

/**
 * This function gets the raw data of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_accel_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_ACCEL_XOUT_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    accel->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    accel->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    accel->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

/**
 * This function gets the raw data of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_gyro_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_GYRO_XOUT_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    gyro->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    gyro->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    gyro->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

#ifdef PKG_USING_MPU6XXX_MAG
/**
 * This function gets the raw data of the magnetometer
 *
 * @param dev the pointer of device driver structure
 * @param mag the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_mag_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *mag)
{
    rt_uint8_t buffer[8];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_EXT_SENS_DATA_00, 8, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    mag->x = ((rt_uint16_t)buffer[2] << 8) + buffer[1];
    mag->y = ((rt_uint16_t)buffer[4] << 8) + buffer[3];
    mag->z = ((rt_uint16_t)buffer[6] << 8) + buffer[5];

    return RT_EOK;
}
#endif

/**
 * This function gets the raw data of the temperature
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_temp_raw(struct mpu6xxx_device *dev, rt_int16_t *temp)
{
    rt_uint8_t buffer[2];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_TEMP_OUT_H, 2, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    *temp = ((rt_uint16_t)buffer[0] << 8) + buffer[1];

    return RT_EOK;
}

/**
 * This function gets mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t *param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate */
        /* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        if (res != RT_EOK)
        {
            break;
        }

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 8000 / (data + 1);
        }
        else /* dlpf is enable */
        {
            res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 1000 / (data + 1);
        }
        break;
    case MPU6XXX_SLEEP: /* sleep mode */
        res = mpu6xxx_read_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, &data);
        *param = data;
        break;
    }

    return res;
}

/**
 * This function set mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mpu6xxx_set_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
        dev->config.gyro_range = param;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
        dev->config.accel_range = param;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate = 16-bit unsigned value.
                                 Sample Rate = [1000 -  4]HZ when dlpf is enable
                                 Sample Rate = [8000 - 32]HZ when dlpf is disable */

        //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        if (res != RT_EOK)
        {
            break;
        }

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            if (param > 8000)
                data = 0;
            else if (param < 32)
                data = 0xFF;
            else
                data = 8000 / param - 1;
        }
        else /* dlpf is enable */
        {
            if (param > 1000)
                data = 0;
            else if (param < 4)
                data = 0xFF;
            else
                data = 1000 / param - 1;
        }
        res = mpu6xxx_write_reg(dev, MPU6XXX_RA_SMPLRT_DIV, data);
        break;
    case MPU6XXX_SLEEP: /* Configure sleep mode */
        res = mpu6xxx_write_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, param);
        break;
    }

    return res;
}

/**
 * This function gets the data of the accelerometer, unit: mg
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_accel(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;
    rt_err_t res;

    res = mpu6xxx_get_accel_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    sen = MPU6XXX_ACCEL_SEN >> dev->config.accel_range;

    accel->x = (rt_int32_t)tmp.x * 1000 / sen;
    accel->y = (rt_int32_t)tmp.y * 1000 / sen;
    accel->z = (rt_int32_t)tmp.z * 1000 / sen;

    return RT_EOK;
}

/**
 * This function gets the data of the gyroscope, unit: deg/10s
 * Here deg/10s means 10 times higher precision than deg/s.
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_gyro(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;
    rt_err_t res;

    res = mpu6xxx_get_gyro_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    sen = MPU6XXX_GYRO_SEN >> dev->config.gyro_range;

    gyro->x = (rt_int32_t)tmp.x * 100 / sen;
    gyro->y = (rt_int32_t)tmp.y * 100 / sen;
    gyro->z = (rt_int32_t)tmp.z * 100 / sen;

    return RT_EOK;
}

#ifdef PKG_USING_MPU6XXX_MAG

/**
 * This function gets the data of the magnetometer, unit: uT
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_mag(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *mag)
{
    struct mpu6xxx_3axes tmp;
    rt_err_t res;

    res = mpu6xxx_get_mag_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    mag->x = ((rt_int32_t)tmp.x * AK8963_RANGE) / AK8963_FULLSCALE;
    mag->y = ((rt_int32_t)tmp.y * AK8963_RANGE) / AK8963_FULLSCALE;
    mag->z = ((rt_int32_t)tmp.z * AK8963_RANGE) / AK8963_FULLSCALE;

    return RT_EOK;
}

#endif

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_temp(struct mpu6xxx_device *dev, float *temp)
{
    rt_int16_t tmp;
    rt_err_t res;

    res = mpu6xxx_get_temp_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    if (dev->id == MPU6050_WHO_AM_I)
    {
        /* mpu60x0: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */
        *temp = (double)tmp / MPU60X0_TEMP_SEN + MPU60X0_TEMP_OFFSET;
    }
    else
    {
        /* mpu6500:  ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)+ 21degC */
        *temp = (double)tmp / MPU6500_TEMP_SEN + MPU6500_TEMP_OFFSET;
    }

    return RT_EOK;
}

/**
* This function sets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  setting the offsets successfully.
 */
rt_err_t mpu6xxx_set_accel_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_err_t res=0;
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XA_OFFS_H, (offset->x)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XA_OFFS_L_TC, (offset->x)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YA_OFFS_H, (offset->y)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YA_OFFS_L_TC, (offset->y)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZA_OFFS_H, (offset->z)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZA_OFFS_L_TC, (offset->z)&0x00ff);
    return res;
}

/**
* This function gets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  reading the offsets successfully.
 */
rt_err_t mpu6xxx_get_accel_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_XA_OFFS_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    offset->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    offset->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    offset->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

return RT_EOK;
}

/**
* This function sets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  setting the offsets successfully.
 */
rt_err_t mpu6xxx_set_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_err_t res=0;
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XG_OFFS_USRH, (offset->x)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XG_OFFS_USRL, (offset->x)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YG_OFFS_USRH, (offset->y)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YG_OFFS_USRL, (offset->y)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZG_OFFS_USRH, (offset->z)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZG_OFFS_USRL, (offset->z)&0x00ff);
    return res;
}

/**
* This function gets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  reading the offsets successfully.
 */
rt_err_t mpu6xxx_get_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_XG_OFFS_USRH, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    offset->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    offset->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    offset->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

return RT_EOK;
}

/**
 * This function initialize the mpu6xxx device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mpu6xxx_device *mpu6xxx_init(const char *dev_name, rt_uint8_t param)
{
    struct mpu6xxx_device *dev = RT_NULL;
    rt_uint8_t reg = 0xFF, res = RT_EOK;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mpu6xxx_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for mpu6xxx device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find mpu6xxx device at address: 0x68 */
            dev->i2c_addr = MPU6XXX_ADDRESS_AD0_LOW;
            if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            {
                /* find mpu6xxx device at address 0x69 */
                dev->i2c_addr = MPU6XXX_ADDRESS_AD0_HIGH;
                if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
                {
                    LOG_E("Can't find device at '%s'!", dev_name);
                    goto __exit;
                }
            }
            LOG_D("Device i2c address is:'0x%x'!", dev->i2c_addr);
        }
    }
    else if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
#ifdef RT_USING_SPI
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = MPU60X0_SPI_MAX_SPEED; /* Set spi max speed */

        rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
#endif
    }
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
    {
        LOG_E("Failed to read device id!");
        goto __exit;
    }

    dev->id = reg;

    switch (dev->id)
    {
    case MPU6050_WHO_AM_I:
        LOG_I("Find device: mpu6050!");
        break;
    case MPU6500_WHO_AM_I:
        LOG_I("Find device: mpu6500!");
        break;
    case MPU9250_WHO_AM_I:
        LOG_I("Find device: mpu9250!");
        break;
    case ICM20608G_WHO_AM_I:
    case ICM20608D_WHO_AM_I:
        LOG_I("Find device: icm20608!");
        break;
    case 0xFF:
        LOG_E("No device connection!");
        goto __exit;
    default:
        LOG_W("Unknown device id: 0x%x!", reg);
    }

    res += mpu6xxx_get_param(dev, MPU6XXX_ACCEL_RANGE, &dev->config.accel_range);
    res += mpu6xxx_get_param(dev, MPU6XXX_GYRO_RANGE, &dev->config.gyro_range);

    res += mpu6xxx_write_bits(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
    res += mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS);
    res += mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);
    res += mpu6xxx_set_param(dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);

#ifdef PKG_USING_MPU6XXX_MAG
    mpu6xxx_write_reg(dev, MPU6XXX_RA_USER_CTRL, 0x20);
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL2, 0x01);      /* [0]  Reset Device                  */
    rt_thread_mdelay(1);
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x00);      /* [1]  Power-down mode               */
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x0F);      /* [2]  Fuse ROM access mode          */
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x00);      /* [3]  Power-down mode               */
    rt_thread_mdelay(1);    // 100us
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x16);      /* [4]  16bits and Continuous measurement mode 2 */

    /* config mpu9250 i2c */
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_MST_CTRL, 0x5D);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_REG, AK8963_REG_ST1);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_CTRL, 0x09);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_MST_DELAY_CTRL, 0x81);
#endif

    if (res == RT_EOK)
    {
        LOG_I("Device init succeed!");
    }
    else
    {
        LOG_W("Error in device initialization!");
    }
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mpu6xxx_deinit(struct mpu6xxx_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mpu6xxx(int argc, char **argv)
{
    static struct mpu6xxx_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mpu6xxx [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mpu6xxx by given name\n");
        rt_kprintf("         sr <var>              Set sample rate to var\n");
        rt_kprintf("                               var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                               var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("         gr <var>              Set gyro range to var\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mpu6xxx\n");
        rt_kprintf("                               num default 5\n");
        return ;
    }
    else if (!strcmp(argv[1], "read"))
    {
        struct mpu6xxx_3axes accel, gyro;
        float temp;
        uint16_t num = 5;

        if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mpu6xxx first!\n");
            return ;
        }
        if (argc == 3)
        {
            num = atoi(argv[2]);
        }

        while (num --)
        {
            mpu6xxx_get_accel(dev, &accel);
            mpu6xxx_get_gyro(dev, &gyro);
            mpu6xxx_get_temp(dev, &temp);

            rt_kprintf("accel.x = %4d, accel.y = %4d, accel.z = %4d ", accel.x, accel.y, accel.z);
            rt_kprintf("gyro.x = %4d gyro.y = %4d, gyro.z = %4d, ", gyro.x, gyro.y, gyro.z);
            rt_kprintf("temp = %d.%d\n", (int)(temp * 100) / 100, (int)(temp * 100) % 100);

            rt_thread_mdelay(100);
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mpu6xxx_deinit(dev);
            }
            dev = mpu6xxx_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mpu6xxx first!\n");
            return ;
        }
        else if (!strcmp(argv[1], "sr"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_SAMPLE_RATE, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "sleep"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_SLEEP, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "gr"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "ar"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, atoi(argv[2]));
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mpu6xxx' get help information!\n");
        }
    }
    else
    {
        rt_kprintf("Unknown command, please enter 'mpu6xxx' get help information!\n");
    }
}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mpu6xxx, mpu6xxx sensor function);
#endif
