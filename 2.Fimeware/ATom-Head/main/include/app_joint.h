#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Joint_config
{
    uint8_t id;
    float angleMin;
    float angleMax;
    float angle;
    float modelAngelMin;
    float modelAngelMax;
    bool inverted;
};

struct Joint_device
{
    struct Joint_config config;
};

void SetJointId(struct Joint_device *_joint, uint8_t _id);

void SetJointKp(struct Joint_device *_joint, float _value);

void SetJointKi(struct Joint_device *_joint, float _value);

void SetJointKv(struct Joint_device *_joint, float _value);

void SetJointKd(struct Joint_device *_joint, float _value);

void SetJointEnable(struct Joint_device *_joint, bool _enable);

void SetJointInitAngle(struct Joint_device *_joint, float _angle);

void SetJointTorqueLimit(struct Joint_device *_joint, float _percent);

void UpdateServoAngle_1(struct Joint_device *_joint);

void UpdateServoAngle_2(struct Joint_device *_joint, float _angleSetPoint);

void UpdateJointAngle_1(struct Joint_device *_joint);

void UpdateJointAngle_2(struct Joint_device *_joint, float _angleSetPoint);

void TransmitAndReceiveI2cPacket(uint8_t _id);

void register_jointcmd(void);

esp_err_t joint_i2c_init(void);

#ifdef __cplusplus
}
#endif