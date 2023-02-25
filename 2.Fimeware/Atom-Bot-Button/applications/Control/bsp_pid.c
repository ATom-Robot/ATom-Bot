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

#include "main.h"
#include "bsp_pid.h"

#define DBG_SECTION_NAME  "PID"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define PID_SCALE  0.01f		//PID缩放系数
#define PID_INTEGRAL_UP 1000	//积分上限

int16_t motor_kp = 10;	//电机转速PID-P
int16_t motor_ki = 0;	//电机转速PID-I
int16_t motor_kd = 0;	//电机转速PID-D

int16_t PID_Motor_Control(int8_t Motor_Num, int16_t speed_target, int16_t speed_current)
{
    static int16_t motor_pwm_out[2];
    static int32_t bias[2], bias_last[2], bias_integral[2] = {0};

    //获得偏差值
    bias[Motor_Num - 1] = speed_target - speed_current;

    //计算偏差累加值
    bias_integral[Motor_Num - 1] += bias[Motor_Num - 1];

    //抗积分饱和
    if (bias_integral[Motor_Num - 1] >  PID_INTEGRAL_UP) bias_integral[Motor_Num - 1] =  PID_INTEGRAL_UP;
    if (bias_integral[Motor_Num - 1] < -PID_INTEGRAL_UP) bias_integral[Motor_Num - 1] = -PID_INTEGRAL_UP;

    //PID计算电机输出PWM值
    motor_pwm_out[Motor_Num - 1] += motor_kp * bias[Motor_Num - 1] * PID_SCALE          \
                                    + motor_kd * (bias[Motor_Num - 1]                   \
									- bias_last[Motor_Num - 1]) * PID_SCALE + motor_ki  \
                                    * bias_integral[Motor_Num - 1] * PID_SCALE;

    //记录上次偏差
    bias_last[Motor_Num - 1] = bias[Motor_Num - 1];

    //限制最大输出
    if (motor_pwm_out[Motor_Num - 1] > 2000)
        motor_pwm_out[Motor_Num - 1] = 2000;
    if (motor_pwm_out[Motor_Num - 1] < -2000)
        motor_pwm_out[Motor_Num - 1] = -2000;

    //返回PWM控制值
    return motor_pwm_out[Motor_Num - 1];
}
