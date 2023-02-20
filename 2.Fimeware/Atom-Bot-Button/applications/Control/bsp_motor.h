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

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <rtthread.h>

#define PWMA1   TIM4->CCR2
#define PWMA2   TIM4->CCR1

#define PWMB1   TIM4->CCR3
#define PWMB2   TIM4->CCR4

#define Dead_Zone	3300

void Motor_Init(void);
void MOTOR_SetSpeed(int8_t Motor_Num, int16_t speed);

#endif

