/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-07-09     Rbb666        First version
 */
#include "bsp_joint.h"
#include "drv_sensor.h"
#include <rtdevice.h>

#define DBG_SECTION_NAME  "JOINT"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define QUEUE_SIZE 512
#define QUEUE_MSG_SIZE sizeof(int)

static uint8_t i2cRxData[8];
static uint8_t i2cTxData[8];
struct Joint_device joint[JOINT_SIZE];

static char mq_pool[QUEUE_SIZE * QUEUE_MSG_SIZE];
static rt_thread_t joint_thread;
static struct rt_messagequeue joint_mq;
TX_QUEUE angle_queue;

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

void UpdateServoAngle_1(struct Joint_device  *_joint)
{
    uint8_t *b = (unsigned char *) & (_joint->config.angle);

    i2cTxData[0] = 0x11;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);
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

rt_bool_t joint_enable_all(rt_bool_t en)
{
    SetJointEnable(&joint[1], en);//right
    SetJointEnable(&joint[2], en);//left

    return en;
}

static int set_init_angle_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_init_angle_to_joint [1: 2] angle\n");
        return RT_ERROR;
    }

    float angle = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointInitAngle(&joint[1], angle);
        rt_kprintf("set joint1 angle:%f\n", angle);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointInitAngle(&joint[2], angle);
        rt_kprintf("set joint2 angle:%f\n", angle);
    }

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
//MSH_CMD_EXPORT(set_init_angle_to_joint, set init angle to joint)

static int set_id_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_id_to_joint [1: 2] id\n");
        return RT_ERROR;
    }

    uint8_t t_id = atoi(argv[1]);
    uint8_t s_id = atoi(argv[2]);

    SetJointId(&joint[t_id], s_id);
    joint[t_id].config.id = s_id;
    rt_kprintf("set joint id:%d\n", s_id);

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
//MSH_CMD_EXPORT(set_id_to_joint, set id to joint)

static int set_kp_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_kp_to_joint [1: 2] kp\n");
        return RT_ERROR;
    }

    float kp = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointKp(&joint[1], kp);
        rt_kprintf("set joint1 kp:%f\n", kp);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointKp(&joint[2], kp);
        rt_kprintf("set joint2 kp:%f\n", kp);
    }

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_kp_to_joint, set kp to joint)

static int set_ki_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_ki_to_joint [1: 2] ki\n");
        return RT_ERROR;
    }

    float ki = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointKi(&joint[1], ki);
        rt_kprintf("set joint1 ki:%f\n", ki);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointKi(&joint[2], ki);
        rt_kprintf("set joint2 ki:%f\n", ki);
    }

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_ki_to_joint, set ki to joint)

static int set_kd_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_kd_to_joint [1: 2] kd\n");
        return RT_ERROR;
    }

    float kd = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointKd(&joint[1], kd);
        rt_kprintf("set joint1 kd:%f\n", kd);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointKd(&joint[2], kd);
        rt_kprintf("set joint2 kd:%f", kd);
    }

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_kd_to_joint, set kd to joint)

static int set_torque_to_joint(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:set_torque_to_joint [1: 2] torque\n");
        return RT_ERROR;
    }

    float torque = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointTorqueLimit(&joint[1], torque);
        rt_kprintf("set joint1 torque:%f\n", torque);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointTorqueLimit(&joint[2], torque);
        rt_kprintf("set joint2 torque:%f\n\n", torque);
    }

    rt_thread_mdelay(1000);
    rt_kprintf("wait 1s to reset joint...\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(set_torque_to_joint, set torque to joint)

static int joint_angle_test(int argc, const char *argv[])
{
    if (argc != 3)
    {
        rt_kprintf("error paramter:joint_angle_test [1: 2] angle\n");
        return RT_ERROR;
    }

    float angle = atof(argv[2]);

    if (rt_strcmp(argv[1], "1") == 0)
    {
        SetJointEnable(&joint[1], RT_TRUE);//right
        UpdateJointAngle_2(&joint[1], angle);
        SetJointEnable(&joint[1], RT_FALSE);//right
        rt_thread_mdelay(100);
        rt_kprintf("set joint1 target angle:%f | joint1 angle:%f\n", angle, joint[1].config.angle);
    }
    else if (rt_strcmp(argv[1], "2") == 0)
    {
        SetJointEnable(&joint[2], RT_TRUE);//right
        UpdateJointAngle_2(&joint[2], angle);
        SetJointEnable(&joint[2], RT_FALSE);//left
        rt_thread_mdelay(100);
        rt_kprintf("set joint2 target angle:%f | joint2 angle:%f\n", angle, joint[2].config.angle);
    }
    else if (rt_strcmp(argv[1], "1&2") == 0)
    {
        joint_enable_all(RT_TRUE);
        UpdateJointAngle_2(&joint[1], angle);
        UpdateJointAngle_2(&joint[2], angle);
        rt_thread_mdelay(100);
        joint_enable_all(RT_FALSE);
        rt_kprintf("set joint1&2 target angle:%f | joint1 angle:%f joint2 angle:%f\n", angle, joint[1].config.angle, joint[2].config.angle);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(joint_angle_test, input: joint_angle_test(1: 2) angle)

void joint_ctrl_entry()
{
    int received_angle;

    while (1)
    {
        if (rt_mq_recv(angle_queue.RT_MQ, &received_angle, sizeof(received_angle), RT_WAITING_FOREVER) > 0)
        {
            sensor_acquire();

            joint_enable_all(RT_TRUE);
            UpdateJointAngle_2(&joint[1], received_angle);
            UpdateJointAngle_2(&joint[2], received_angle);
            rt_thread_mdelay(50);
            joint_enable_all(RT_FALSE);

            sensor_release();
        }
    }
}

int joint_thread_create(void)
{
    joint_thread = rt_thread_create("joint",
                                    joint_ctrl_entry,
                                    RT_NULL,
                                    1024,
                                    25,
                                    20);
    if (joint_thread != RT_NULL)
    {
        rt_thread_startup(joint_thread);
    }

    rt_mq_init(&joint_mq, "angle_mq", &mq_pool[0], QUEUE_MSG_SIZE, sizeof(mq_pool), RT_IPC_FLAG_PRIO);
    angle_queue.RT_MQ = &joint_mq;

    return RT_EOK;
}

static int joint_init(void)
{
    joint[ANY].config.id = 0;
    joint[ANY].config.angleMin = 0;
    joint[ANY].config.angleMax = 90;
    joint[ANY].config.angle = 0;
    joint[ANY].config.modelAngelMin = -90;
    joint[ANY].config.modelAngelMax = 90;
    joint[ANY].config.inverted = RT_FALSE;

    // right
    joint[1].config.id = 1;
    joint[1].config.angleMin = 0;
    joint[1].config.angleMax = 90;
    joint[1].config.angle = 0;
    joint[1].config.modelAngelMin = -20;
    joint[1].config.modelAngelMax = 90;
    joint[1].config.inverted = RT_FALSE;

    // left
    joint[2].config.id = 2;
    joint[2].config.angleMin = 0;
    joint[2].config.angleMax = 90;
    joint[2].config.angle = 0;
    joint[2].config.modelAngelMin = -20;
    joint[2].config.modelAngelMax = 90;
    joint[2].config.inverted = RT_TRUE;

    joint_enable_all(RT_TRUE);

    UpdateJointAngle_2(&joint[1], 0);
    UpdateJointAngle_2(&joint[2], 0);

    rt_thread_mdelay(500);

    joint_enable_all(RT_FALSE);

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
            rt_kprintf("Can't find device:'%s'\n", joint[i].bus);
            res = RT_ERROR;
            break;
        }
    }

    joint_init();
    joint_thread_create();

    return res;
}
