/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#include "BSP_Joint.h"
#include <rtdevice.h>

#define DBG_SECTION_NAME  "JOINT"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static uint8_t i2cRxData[8];
static uint8_t i2cTxData[8];
struct Joint_device joint[JOINT_SIZE];

static rt_err_t joint_write_regs(struct Joint_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs[0].addr  = dev->config.id;   /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->config.id;   /* Slave address */
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
    }
    return res;
}

static rt_err_t joint_write_reg(struct Joint_device *dev, uint8_t *pData, rt_uint8_t len)
{
    int8_t res = 0;
    struct rt_i2c_msg msgs;
    uint8_t buf[5] = {0};
    rt_memcpy((uint8_t *)buf, (uint8_t *)pData, len);

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = dev->config.id;    /* slave address */
        msgs.flags = RT_I2C_WR;         /* write flag */
        msgs.buf   = buf;               /* Send data pointer */
        msgs.len   = len;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
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

static rt_err_t joint_read_regs(struct Joint_device *dev, rt_uint8_t len, rt_uint8_t *buf)
{
    int8_t res = 0;
    struct rt_i2c_msg msgs;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = dev->config.id;    /* Slave address */
        msgs.flags = RT_I2C_RD;         /* Read flag */
        msgs.buf   = buf;               /* Read data pointer */
        msgs.len   = len;               /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
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

void UpdateServoAngle_2(struct Joint_device  *_joint, float _angleSetPoint)
{
    if (_angleSetPoint >= _joint->config.angleMin && _angleSetPoint <= _joint->config.angleMax)
    {
        uint8_t *b = (unsigned char *)(&_angleSetPoint);

        i2cTxData[0] = 0x01;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint);

        _joint->config.angle = *(float *)(i2cRxData + 1);
    }
}

void UpdateServoAngle_1(struct Joint_device  *_joint)
{
    uint8_t *b = (unsigned char *) & (_joint->config.angle);

    i2cTxData[0] = 0x11;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointEnable(struct Joint_device  *_joint, rt_bool_t _enable)
{
    i2cTxData[0] = 0xff;
    i2cTxData[1] = _enable ? 1 : 0;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void TransmitAndReceiveI2cPacket(struct Joint_device *_joint)
{
    joint_write_reg(_joint, i2cTxData, 5);
    joint_read_regs(_joint, 5, i2cRxData);
}

void SetJointTorqueLimit(struct Joint_device  *_joint, float _percent)
{
    if (_percent >= 0 && _percent <= 1)
    {
        uint8_t *b = (unsigned char *)(&_percent);

        i2cTxData[0] = 0x26;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint);

        _joint->config.angle = *(float *)(i2cRxData + 1);

        rt_thread_mdelay(500); // wait servo reset
    }
}

void SetJointId(struct Joint_device  *_joint, uint8_t _id)
{
    i2cTxData[0] = 0x21;
    i2cTxData[1] = _id;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointInitAngle(struct Joint_device  *_joint, float _angle)
{
    float sAngle = _joint->config.inverted ?
                   (_angle - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMin - _joint->config.angleMax) + _joint->config.angleMax :
                   (_angle - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMax - _joint->config.angleMin) + _joint->config.angleMin;


    if (sAngle >= _joint->config.angleMin && sAngle <= _joint->config.angleMax)
    {
        uint8_t *b = (unsigned char *)(&_angle);

        i2cTxData[0] = 0x27;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint);

        _joint->config.angle = *(float *)(i2cRxData + 1);

        rt_thread_mdelay(500); // wait servo reset
    }
}

void SetJointKp(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x22;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKi(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x23;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKv(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x24;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKd(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x25;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void UpdateJointAngle_1(struct Joint_device  *_joint)
{
    UpdateServoAngle_1(_joint);

    float jAngle = _joint->config.inverted ?
                   (_joint->config.angleMax - _joint->config.angle) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin :
                   (_joint->config.angle - _joint->config.angleMin) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin;

    _joint->config.angle = jAngle;
}

void UpdateJointAngle_2(struct Joint_device  *_joint, float _angleSetPoint)
{
    float sAngle = _joint->config.inverted ?
                   (_angleSetPoint - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMin - _joint->config.angleMax) + _joint->config.angleMax :
                   (_angleSetPoint - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMax - _joint->config.angleMin) + _joint->config.angleMin;

    UpdateServoAngle_2(_joint, sAngle);

    float jAngle = _joint->config.inverted ?
                   (_joint->config.angleMax - _joint->config.angle) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin :
                   (_joint->config.angle - _joint->config.angleMin) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin;

    _joint->config.angle = jAngle;
}

static int set_id_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:set_id_to_joint [1: 2] id");
        return RT_ERROR;
    }

    uint8_t t_id = atoi(argv[1]);
    uint8_t s_id = atoi(argv[2]);

	SetJointEnable(&joint[t_id], RT_FALSE);
	SetJointId(&joint[t_id], s_id);
	SetJointEnable(&joint[t_id], RT_TRUE);
	joint[t_id].config.id = s_id;
	LOG_D("set joint id:%d", s_id);

    rt_thread_mdelay(1000);
	LOG_D("wait 1s to reset joint...");

    return RT_EOK;
}
//MSH_CMD_EXPORT(set_id_to_joint, set id to joint)

static int set_kp_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:set_kp_to_joint [1: 2] kp");
        return RT_ERROR;
    }

    float kp = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
		SetJointEnable(&joint[1], RT_FALSE);
        SetJointKp(&joint[1], kp);
		SetJointEnable(&joint[1], RT_TRUE);
        LOG_D("set joint1 kp:%f", kp);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
		SetJointEnable(&joint[2], RT_FALSE);
        SetJointKp(&joint[2], kp);
		SetJointEnable(&joint[2], RT_TRUE);
        LOG_D("set joint2 kp:%f", kp);
    }

    rt_thread_mdelay(1000);
	LOG_D("wait 1s to reset joint...");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_kp_to_joint, set kp to joint)

static int set_ki_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:set_ki_to_joint [1: 2] ki");
        return RT_ERROR;
    }

    float ki = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
		SetJointEnable(&joint[1], RT_FALSE);
        SetJointKi(&joint[1], ki);
		SetJointEnable(&joint[1], RT_TRUE);
        LOG_D("set joint1 ki:%f", ki);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
		SetJointEnable(&joint[2], RT_FALSE);
        SetJointKi(&joint[2], ki);
		SetJointEnable(&joint[2], RT_TRUE);
        LOG_D("set joint2 ki:%f", ki);
    }

    rt_thread_mdelay(1000);
	LOG_D("wait 1s to reset joint...");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_ki_to_joint, set ki to joint)

static int set_kd_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:set_kd_to_joint [1: 2] kd");
        return RT_ERROR;
    }

    float kd = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
		SetJointEnable(&joint[1], RT_FALSE);
        SetJointKd(&joint[1], kd);
		SetJointEnable(&joint[1], RT_TRUE);
        LOG_D("set joint1 kd:%f", kd);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
		SetJointEnable(&joint[2], RT_FALSE);
        SetJointKd(&joint[2], kd);
		SetJointEnable(&joint[2], RT_TRUE);
        LOG_D("set joint2 kd:%f", kd);
    }

    rt_thread_mdelay(1000);
	LOG_D("wait 1s to reset joint...");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_kd_to_joint, set kd to joint)

static int set_torque_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:set_torque_to_joint [1: 2] torque");
        return RT_ERROR;
    }

    float torque = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
		SetJointEnable(&joint[1], RT_FALSE);
        SetJointTorqueLimit(&joint[1], torque);
		SetJointEnable(&joint[1], RT_TRUE);
        LOG_D("set joint1 torque:%f", torque);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
		SetJointEnable(&joint[2], RT_FALSE);
        SetJointTorqueLimit(&joint[2], torque);
		SetJointEnable(&joint[2], RT_TRUE);
        LOG_D("set joint2 torque:%f", torque);
    }

    rt_thread_mdelay(1000);
	LOG_D("wait 1s to reset joint...");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_torque_to_joint, set torque to joint)

static int joint_angle_test(int argc, const char *argv[])
{
    if (argc != 3)
    {
        LOG_E("error paramter:joint_angle_test [1: 2] angle");
        return RT_ERROR;
    }

    float angle = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        UpdateJointAngle_2(&joint[1], angle);
        LOG_D("set joint1 target angle:%f | joint1 angle:%f", angle, joint[1].config.angle);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        UpdateJointAngle_2(&joint[2], angle);
        LOG_D("set joint2 target angle:%f | joint2 angle:%f", angle, joint[2].config.angle);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(joint_angle_test, input: joint_angle_test(1: 2) angle)

static int joint_init(void)
{
    joint[ANY].config.id = 0;
    joint[ANY].config.angleMin = 0;
    joint[ANY].config.angleMax = 180;
    joint[ANY].config.angle = 0;
    joint[ANY].config.modelAngelMin = -90;
    joint[ANY].config.modelAngelMax = 90;
    joint[ANY].config.inverted = RT_FALSE;

    //right
    joint[1].config.id = 1;
    joint[1].config.angleMin = 0;
    joint[1].config.angleMax = 95;
    joint[1].config.angle = 0;
    joint[1].config.modelAngelMin = -20;
    joint[1].config.modelAngelMax = 90;
    joint[1].config.inverted = RT_FALSE;

    //left
    joint[2].config.id = 2;
    joint[2].config.angleMin = 0;
    joint[2].config.angleMax = 90;
    joint[2].config.angle = 0;
    joint[2].config.modelAngelMin = -90;
    joint[2].config.modelAngelMax = 0;
    joint[2].config.inverted = RT_FALSE;

    SetJointEnable(&joint[1], RT_TRUE);//right
    SetJointEnable(&joint[2], RT_TRUE);//left

    UpdateJointAngle_2(&joint[1], 0);
    UpdateJointAngle_2(&joint[2], 0);

    return RT_EOK;
}

int joint_i2c_init(void)
{
    rt_uint8_t res = RT_EOK;

    for (int i = 0; i < JOINT_SIZE; i++)
    {
        joint[i].bus = rt_device_find("i2c2");
        if (joint[i].bus == RT_NULL)
        {
            LOG_D("Can't find device:'%s'", joint[i].bus);
            res = -RT_ERROR;
            break;
        }
    }

    rt_thread_mdelay(100);

    joint_init();

    return res;
}
