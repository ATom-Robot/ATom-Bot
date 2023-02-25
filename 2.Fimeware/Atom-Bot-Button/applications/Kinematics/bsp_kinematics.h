/*
    Copyright 2022 Fan Ziqi

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef BSP_KINEMATICS_H
#define BSP_KINEMATICS_H

#include <rtthread.h>

#define ENCODER_RESOLUTION	7.0		//编码器分辨率, 轮子转一圈，编码器产生的脉冲数(编码器分辨率：12*30*4（12线，1：30减速比，四倍频）)
#define WHEEL_DIAMETER		0.017	//轮子直径,单位：米
#define D_X					0.045	//底盘Y轴上两轮中心的间距
#define D_Y					0.059	//底盘X轴上两轮中心的间距
#define PID_RATE			50		//PID调节PWM值的频率(50HZ=20ms)

#define ROBOT_LINEAR_SPEED_LIMIT 5000		//机器人线速度限值 m/s*1000
#define ROBOT_ANGULAR_SPEED_LIMIT 10000		//机器人角速度限值 rad/s*1000

#define ENCODER_MAX 32767
#define ENCODER_MIN -32768
#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN) * 0.3 + ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN) * 0.7 + ENCODER_MIN)
#define PI 3.1415926

//ROBOT功能函数
void Kinematics_Init(int16_t *robot_params);  //参数初始化
void Kinematics_Forward(int16_t *input, int16_t *output); //正解(ForwardKinematics)：轮子编码值->底盘三轴里程计坐标
void Kinematics_Inverse(int16_t *input, int16_t *output); //逆解(InverseKinematics)：底盘三轴速度->轮子速度


#endif
