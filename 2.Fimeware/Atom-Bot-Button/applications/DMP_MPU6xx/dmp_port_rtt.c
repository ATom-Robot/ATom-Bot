/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66			First version
 */
#include <rtdevice.h>
#include <math.h>
#include <AT_Math.h>
#include "mpu6xxx.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmp_port_rtt.h"

#define MPU6050_I2C_BUS_NAME    "i2c2"
#define MPU6050_ADDR    RT_NULL

#define DBG_SECTION_NAME  "dmp_port"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static struct mpu6xxx_device *i2c_bus = RT_NULL;

//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };
static short sensors;
static float pitch, roll, yaw;

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *databuf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    uint8_t buf[50] = {0};
#endif
    buf[0] = reg;

    for (int i = 0; i < len; i++)
    {
        buf[i + 1] = databuf[i];
    }

    if (i2c_bus->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = i2c_bus->i2c_addr; /* slave address */
        msgs.flags = RT_I2C_WR;         /* write flag */
        msgs.buf   = buf;               /* Send data pointer */
        msgs.len   = len + 1;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)i2c_bus->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
    }
    return res;
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    uint8_t tmp;
#endif
    if (i2c_bus->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs[0].addr  = i2c_bus->i2c_addr;  /* Slave address */
        msgs[0].flags = RT_I2C_WR;          /* Write flag */
        msgs[0].buf   = &reg;               /* Slave register address */
        msgs[0].len   = 1;                  /* Number of bytes sent */

        msgs[1].addr  = i2c_bus->i2c_addr;  /* Slave address */
        msgs[1].flags = RT_I2C_RD;          /* Read flag */
        msgs[1].buf   = buf;                /* Read data pointer */
        msgs[1].len   = len;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)i2c_bus->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
    }

    return res;
}

static void MPU6050_Write_Byte(uint8_t reg_address, uint8_t data)
{
    MPU_Write_Len(MPU6050_ADDR, reg_address, 1, &data);
}

static void MPU6050_Read_Byte(uint8_t reg_address, uint8_t *pdata)
{
    MPU_Read_Len(MPU6050_ADDR, reg_address, 1, pdata);
}

/**
  * @简  述  设置MPU6050加速度传感器量程
  * @参  数  range： 加速度量程 0,±2g;1,±4g;2,±8g;3,±16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void MPU6050_SetAccelRange(uint8_t range)
{
    MPU6050_Write_Byte(MPU6050_ACCEL_CFG_REG, range << 3);
}

/**
  * @简  述  设置MPU6050陀螺仪传感器满量程范围
  * @参  数  range 陀螺仪量程 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps (degree per second)
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000
  * @返回值  无
  */
void MPU6050_SetGyroRange(uint8_t range)
{
    MPU6050_Write_Byte(MPU6050_GYRO_CFG_REG, range << 3);
}

/**
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  lpf:数字低通滤波频率(Hz)
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void MPU6050_SetDLPF(uint8_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)
        data = DLPF_ACC184_GYRO188;
    else if (lpf >= 98)
        data = DLPF_ACC94_GYRO98;
    else if (lpf >= 42)
        data = DLPF_ACC44_GYRO42;
    else if (lpf >= 20)
        data = DLPF_ACC21_GYRO20;
    else if (lpf >= 10)
        data = DLPF_ACC10_GYRO10;
    else
        data = DLPF_ACC5_GYRO5;
    MPU6050_Write_Byte(MPU6050_CFG_REG, data); //设置数字低通滤波器
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz
  * @返回值  无
  */
void MPU6050_SetSmplRate(uint16_t smplrate)
{
    if (smplrate > 1000)
        smplrate = 1000;
    if (smplrate < 10)
        smplrate = 10;

    MPU6050_Write_Byte(MPU6050_SAMPLE_RATE_REG, (uint8_t)(1000 / smplrate - 1)); //设置数字低通滤波器
    MPU6050_SetDLPF(smplrate / 2);  //自动设置LPF为采样率的一半
}

//方向转换
static uint16_t inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;  // error
    return b;
}

//陀螺仪方向控制
static uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*
    XYZ  010_001_000 Identity Matrix
    XZY  001_010_000
    YXZ  010_000_001
    YZX  000_010_001
    ZXY  001_000_010
    ZYX  000_001_010
    */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

//MPU6050自测试
//返回值:0,正常
//    其他,失败
static uint8_t run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);

    if (result == 0x3)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;

        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    }
    else return 1;
}

/**
  * @简  述  MPU6050获取传感器温度值
  * @参  数  无
  * @返回值  传感器温度值。
  */
float MPU6050_GetTempValue(void)
{
    uint8_t buf[2];
    int16_t tmp;

    MPU_Read_Len(MPU6050_ADDR, MPU6050_TEMP_OUTH_REG, 2, buf);

    tmp = (buf[0] << 8) | buf[1];

    return (36.53f + ((double)tmp / 340.0f));
}

/**
  * @简  述  MPU6050获取X轴加速度寄存器输出值
  * @参  数  无
  * @返回值  X轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_X(void)
{
    uint8_t buf[2];

    MPU_Read_Len(MPU6050_ADDR, MPU6050_ACCEL_XOUTH_REG, 2, buf);

    return ((buf[0] << 8) | buf[1]);
}

/**
  * @简  述  MPU6050获取Y轴加速度寄存器输出值
  * @参  数  无
  * @返回值  Y轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_Y(void)
{
    uint8_t buf[2];

    MPU_Read_Len(MPU6050_ADDR, MPU6050_ACCEL_YOUTH_REG, 2, buf);

    return ((buf[0] << 8) | buf[1]);
}

/**
  * @简  述  MPU6050获取Z轴加速度寄存器输出值
  * @参  数  无
  * @返回值  Z轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_Z(void)
{
    uint8_t buf[2];

    MPU_Read_Len(MPU6050_ADDR, MPU6050_ACCEL_ZOUTH_REG, 2, buf);

    return ((buf[0] << 8) | buf[1]);
}

/**
  * @简  述  MPU6050获取三轴加速度寄存器输出值(带符号的原始值)
  * @参  数  pbuf：读取的数据缓冲区指针
  * @返回值  无
  */
void MPU6050_GetAccelData(struct xyz_data *robot_accel_xyz_data)
{
    uint8_t buf[6];

    MPU_Read_Len(MPU6050_ADDR, MPU6050_ACCEL_XOUTH_REG, 6, buf);

    robot_accel_xyz_data->x = (buf[0] << 8) | buf[1];
    robot_accel_xyz_data->y = (buf[2] << 8) | buf[3];
    robot_accel_xyz_data->z = (buf[4] << 8) | buf[5];
}

void MPU6050_DMP_GetData(struct imu_data *robot_imu_data)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

    robot_imu_data->gyro.x = gyro[0];
    robot_imu_data->gyro.y = gyro[1];
    robot_imu_data->gyro.z = gyro[2];

    robot_imu_data->accel.x = accel[0];
    robot_imu_data->accel.y = accel[1];
    robot_imu_data->accel.z = accel[2];

    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
    **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
    **/

    if (sensors & INV_WXYZ_QUAT)
    {
        //q30格式转换为浮点数
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        float t_temp;
        t_temp = LIMIT(1 - my_pow(-2 * q1 * q3 + 2 * q0 * q2), 0, 1);

        //计算得到俯仰角/横滚角/航向角
        pitch = fast_atan2((-2 * q1 * q3 + 2 * q0 * q2), my_sqrt(t_temp)) * 57.3f; // pitch
        roll  = fast_atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f; // roll
        yaw   = fast_atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f; //yaw

        robot_imu_data->pitch = pitch * 100;
        robot_imu_data->roll  = roll  * 100;
        robot_imu_data->yaw   = yaw   * 100;
    }
}

int MPU6050_DMP_Init(void)
{
	if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		LOG_I("mpu设置传感器完成 ......");
	else
	{
		LOG_E("mpu设置传感器失败");
		return RT_ERROR;
	}
	if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		LOG_I("mpu配置FIFO完成 ......");
	else
	{
		LOG_E("mpu配置FIFO失败");
		return RT_ERROR;
	}
	if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		LOG_I("mpu设定的采样率完成 ......");
	else
	{
		LOG_E("mpu设定的采样率失败");
		return RT_ERROR;
	}
	if (!dmp_load_motion_driver_firmware())
		LOG_I("DMP加载动作固件完成 ......");
	else
	{
		LOG_E("DMP加载动作固件失败");
		return RT_ERROR;
	}
	if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		LOG_I("DMP设置陀螺仪方向完成 ......");
	else
	{
		LOG_E("mpu设置传感器失败");
		return RT_ERROR;
	}
	if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
							DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
							DMP_FEATURE_GYRO_CAL))
		LOG_I("DMP功能已使能 ......");
	else
	{
		LOG_E("DMP使能失败");
		return RT_ERROR;
	}
	if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		LOG_I("DMP设定FIFO速率完成 ......");
	else
	{
		LOG_E("DMP设定FIFO速率失败");
		return RT_ERROR;
	}
	run_self_test();
	if (!mpu_set_dmp_state(1))
		LOG_I("mpu设置DMP状态完成 ......");
	else
	{
		LOG_E("mpu设置DMP状态失败");
		return RT_ERROR;
	}

    return RT_EOK;
}

/**
  * @简  述  MPU6050传感器初始化
  * @参  数  无
  * @返回值  无
  */
int MPU6050_Init(void)
{
	rt_err_t res = RT_EOK;

    i2c_bus = (struct mpu6xxx_device *)mpu6xxx_init(MPU6050_I2C_BUS_NAME, MPU6050_ADDR);
    RT_ASSERT(i2c_bus != RT_NULL);

    if (!mpu_init())
    {
		res = MPU6050_DMP_Init();
		RT_ASSERT(res != RT_ERROR);
    }

    return res;
}
MSH_CMD_EXPORT(MPU6050_Init, MPU6050 DMP Init);

rt_err_t Read_mpu6xx_dmp(int argc, const char *argv[])
{
    struct imu_data robot_imu_dmp_data;

    if (argc != 2)
    {
        LOG_E("ERROR Paramter!");
        return RT_ERROR;
    }

    int loop_cnt = atoi(argv[1]);

    for (int i = 0; i < loop_cnt; i++)
    {
        //读取DMP数据 251 2.51
        MPU6050_DMP_GetData(&robot_imu_dmp_data);
        LOG_I("%d.%d %d.%d %d.%d \r\n",
              robot_imu_dmp_data.pitch / 100, abs(robot_imu_dmp_data.pitch / 10 % 10),
              robot_imu_dmp_data.roll / 100, abs(robot_imu_dmp_data.roll / 10 % 10),
              robot_imu_dmp_data.yaw / 100, abs(robot_imu_dmp_data.yaw / 10 % 10));

        rt_thread_mdelay(50);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(Read_mpu6xx_dmp, Read data mpu6050 from dmp);
