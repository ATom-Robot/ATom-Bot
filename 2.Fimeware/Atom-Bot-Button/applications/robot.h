#ifndef  __ROBOT_H__
#define  __ROBOT_H__

#include <rtthread.h>
#include <chassis.h>

#include "dual_pwm_motor.h"
#include "ab_phase_encoder.h"
#include "inc_pid_controller.h"

// MOTOR
#define LEFT_FORWARD_PWM            "pwm4"
#define LEFT_FORWARD_PWM_CHANNEL    1			// GPIO PD12

#define LEFT_BACKWARD_PWM           "pwm4"   
#define LEFT_BACKWARD_PWM_CHANNEL   2			// GPIO PB8

#define RIGHT_FORWARD_PWM           "pwm4"
#define RIGHT_FORWARD_PWM_CHANNEL   3			// GPIO PA0

#define RIGHT_BACKWARD_PWM          "pwm4"
#define RIGHT_BACKWARD_PWM_CHANNEL   4			// GPIO PB10

// ENCODER
#define LEFT_FORWARD_ENCODER_PIN_A     0		// GET_PIN(A, 0)
#define RIGHT_FORWARD_ENCODER_PIN_A    1		// GET_PIN(A, 1)

#define LEFT_FORWARD_ENCODER_PIN_B     6		// GET_PIN(A, 6)
#define RIGHT_FORWARD_ENCODER_PIN_B    7		// GET_PIN(A, 7)

#define PULSE_PER_REVOL             7			// Real value 20
#define ENCODER_SAMPLE_TIME			50

// CONTROLLER PID
#define PID_SAMPLE_TIME             50
#define PID_PARAM_KP                6
#define PID_PARAM_KI                0
#define PID_PARAM_KD                0

// WHEEL
#define WHEEL_RADIUS             0.066
#define GEAR_RATIO                  50

#define WHEEL_DIST_X                 0
#define WHEEL_DIST_Y              0.05

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
    inc_pid_controller_t left_forward_pid, right_forward_pid;
    wheel_t left_forward_wheel, right_forward_wheel;

    motor_t x_servo, y_servo;
    kinematics_t c_kinematics;
    E_ROC_ROBOT_STATUS status;

    rt_int8_t speed ;
    rt_int16_t degree;

} ST_ROC_ROBOT;

#endif
