#ifndef  __ROBOT_H__
#define  __ROBOT_H__

#include <rtthread.h>

#include "ano.h"
#include "ab_encoder.h"
#include "dual_pwm_motor.h"

// MOTOR
#define MOTOR_PWM						"pwm4"
#define RIGHT_FORWARD_PWM_CHANNEL		1		// GPIO PB6
#define RIGHT_BACKWARD_PWM_CHANNEL		2		// GPIO PB7
#define LEFT_FORWARD_PWM_CHANNEL		3		// GPIO PB8
#define LEFT_BACKWARD_PWM_CHANNEL		4		// GPIO PB9

// ENCODER
#define LEFT_FORWARD_ENCODER_PIN_A		0		// GET_PIN(A, 0)
#define LEFT_FORWARD_ENCODER_PIN_B		1		// GET_PIN(A, 1)

#define RIGHT_FORWARD_ENCODER_PIN_A		6		// GET_PIN(A, 6)
#define RIGHT_FORWARD_ENCODER_PIN_B		7		// GET_PIN(A, 7)

#define PULSE_PER_REVOL             7			// Real value 7
#define ENCODER_SAMPLE_TIME			50

/* Maximum PWM signal */
#define MIN_PWM						-1000
#define MAX_PWM						1000

#define ENABLE_DBG					1

typedef enum
{
    E_FORWARD = 0,
    E_BACK,
    E_LEFT,
    E_RIGHT,
    E_RIGHT_ROTATE,
    E_LEFT_ROTATE,
    E_STOP,
    E_RUNNING,
    E_SPEED_UP,
    E_SPEED_DOWN,
    E_LOW_POWER,
} E_ROC_ROBOT_STATUS;

typedef struct
{
	dual_pwm_motor_t left_forward_motor, right_forward_motor;
    ab_phase_encoder_t left_forward_encoder, right_forward_encoder;

    E_ROC_ROBOT_STATUS status;

} ROBOT;

ROBOT robot;

#endif
