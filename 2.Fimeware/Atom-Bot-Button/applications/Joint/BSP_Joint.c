#include "BSP_Joint.h"
#include <rtdevice.h>

#define DBG_SECTION_NAME  "JOINT"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static uint8_t i2cRxData[8];
static uint8_t i2cTxData[8];
struct Joint_device joint[JOINT_SIZE];

static rt_err_t joint_write_reg(struct Joint_device *dev, uint8_t *pData, rt_uint8_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;
    uint8_t buf[5] = {0};
	rt_memcpy((uint8_t *)buf, (uint8_t *)pData, len);

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = dev->config.id;		/* slave address */
        msgs.flags = RT_I2C_IGNORE_NACK;	/* write flag */
        msgs.buf   = buf;					/* Send data pointer */
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
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = dev->config.id;    /* Slave address */
        msgs.flags = RT_I2C_RD;			/* Read flag */
        msgs.buf   = buf;				/* Read data pointer */
        msgs.len   = len;				/* Number of bytes read */

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

void UpdateServoAngle_2(struct Joint_device*  _joint, float _angleSetPoint)
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

void UpdateServoAngle_1(struct Joint_device*  _joint)
{
    uint8_t *b = (unsigned char *) & (_joint->config.angle);

    i2cTxData[0] = 0x11;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointEnable(struct Joint_device*  _joint, rt_bool_t _enable)
{
    i2cTxData[0] = 0xff;
    i2cTxData[1] = _enable ? 1 : 0;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void TransmitAndReceiveI2cPacket(struct Joint_device* _joint)
{
    rt_err_t state = -RT_ERROR;

	state = joint_write_reg(_joint, i2cTxData, 5);

	state = joint_read_regs(_joint, 5, i2cRxData);
}

void SetJointTorqueLimit(struct Joint_device*  _joint, float _percent)
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

void SetJointId(struct Joint_device*  _joint, uint8_t _id)
{
    i2cTxData[0] = 0x21;
    i2cTxData[1] = _id;

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointInitAngle(struct Joint_device*  _joint, float _angle)
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

void SetJointKp(struct Joint_device*  _joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x22;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKi(struct Joint_device*  _joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x23;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKv(struct Joint_device*  _joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x24;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void SetJointKd(struct Joint_device*  _joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x25;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    rt_thread_mdelay(500); // wait servo reset
}

void UpdateJointAngle_1(struct Joint_device*  _joint)
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

void UpdateJointAngle_2(struct Joint_device*  _joint, float _angleSetPoint)
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

int set_id_to_joint()
{
    joint[ANY].config.id = 0;
    joint[ANY].config.angleMin = 0;
    joint[ANY].config.angleMax = 180;
    joint[ANY].config.angle = 0;
    joint[ANY].config.modelAngelMin = -90;
    joint[ANY].config.modelAngelMax = 90;
    SetJointId(&joint[ANY], 4);
	
	rt_thread_mdelay(1000);
	joint[ANY].config.id = 2;
	SetJointEnable(&joint[ANY], RT_TRUE);
	
	return RT_EOK;
}
MSH_CMD_EXPORT(set_id_to_joint, set_id_to_joint)

int joint_init(void)
{
	// Left arm
	joint[ANY].config.id = 2;
	joint[ANY].config.angleMin = 0;
	joint[ANY].config.angleMax = 180;
	joint[ANY].config.angle = 0;
	joint[ANY].config.modelAngelMin = -90;
	joint[ANY].config.modelAngelMax = 90;
	joint[ANY].config.inverted = RT_FALSE;

	UpdateJointAngle_2(&joint[ANY], 0);
	SetJointEnable(&joint[ANY], RT_TRUE);

	return RT_EOK;
}

int joint_angele_test(int argc, const char*argv[])
{
	if (argc != 2)
	{
		LOG_E("ERROR Paramter");
		return RT_ERROR;
	}

	float angle = atof(argv[1]);

	UpdateJointAngle_2(&joint[ANY], angle);
	LOG_I("set angle:%f | target angle:%f", angle, joint[ANY].config.angle);	

	return RT_EOK;
}
MSH_CMD_EXPORT(joint_angele_test, joint test:-180-180)

int joint_i2s_init(void)
{
    rt_uint8_t res = RT_EOK;

	for (int i = 0; i < JOINT_SIZE; i++)
	{
		joint[i].bus = rt_device_find("i2c2");
		if (joint[i].bus == RT_NULL)
		{
			LOG_I("Can't find device:'%s'", "i2c2");
			res = -RT_ERROR;
			break;
		}
	}
	
	rt_thread_mdelay(1000);

	joint_init();

	return res;
}
