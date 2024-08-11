#ifndef BSP_JOINT_H
#define BSP_JOINT_H

#include <rtthread.h>
#include <stdint.h>

#define ANY 0
#define JOINT_SIZE 3

struct Joint_config
{
    uint8_t id;
    float angleMin;
    float angleMax;
    float angle;
    float modelAngelMin;
    float modelAngelMax;
    rt_bool_t inverted;
};

/* mpu6xxx device structure */
struct Joint_device
{
    rt_device_t bus;
    struct Joint_config config;
};

typedef struct {
    rt_mq_t RT_MQ;
} TX_QUEUE;

extern TX_QUEUE angle_queue;

rt_bool_t joint_enable_all(rt_bool_t en);

void SetJointId(struct Joint_device* _joint, uint8_t _id);

void SetJointKp(struct Joint_device* _joint, float _value);

void SetJointKi(struct Joint_device* _joint, float _value);

void SetJointKv(struct Joint_device* _joint, float _value);

void SetJointKd(struct Joint_device* _joint, float _value);

void SetJointEnable(struct Joint_device* _joint, rt_bool_t _enable);

void SetJointInitAngle(struct Joint_device* _joint, float _angle);

void SetJointTorqueLimit(struct Joint_device* _joint, float _percent);

void UpdateServoAngle_1(struct Joint_device* _joint);

void UpdateServoAngle_2(struct Joint_device* _joint, float _angleSetPoint);

void UpdateJointAngle_1(struct Joint_device* _joint);

void UpdateJointAngle_2(struct Joint_device* _joint, float _angleSetPoint);

void TransmitAndReceiveI2cPacket(struct Joint_device* _joint);

int joint_i2c_init(void);

#endif //ELECTRONBOT_FW_ROBOT_H
