#include <rtthread.h>
#include "robot.h"

#define DBG_SECTION_NAME  "robot"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

chassis_t chas;
ST_ROC_ROBOT roc_robot;

void car_thread(void *param);

int robot_init(void *parameter)
{
    wheel_t *c_wheels = (wheel_t *) rt_malloc(sizeof(wheel_t) * 2);

    if (c_wheels == RT_NULL)
    {
        LOG_E("Failed to malloc memory for wheels");
    }

	// 1.1 Create two motors
    roc_robot.left_forward_motor = dual_pwm_motor_create(LEFT_FORWARD_PWM, LEFT_FORWARD_PWM_CHANNEL, LEFT_FORWARD_PWM, LEFT_BACKWARD_PWM_CHANNEL);
    roc_robot.right_forward_motor = dual_pwm_motor_create(RIGHT_FORWARD_PWM, RIGHT_FORWARD_PWM_CHANNEL, RIGHT_FORWARD_PWM, RIGHT_BACKWARD_PWM_CHANNEL);

    // 1.2 Create two encoders
    roc_robot.left_forward_encoder  = ab_phase_encoder_create(LEFT_FORWARD_ENCODER_PIN_A, RIGHT_FORWARD_ENCODER_PIN_A, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
    roc_robot.right_forward_encoder  = ab_phase_encoder_create(LEFT_FORWARD_ENCODER_PIN_B, RIGHT_FORWARD_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
	
	// 1.3 Create two pid contollers
    roc_robot.left_forward_pid  = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);
    roc_robot.right_forward_pid  = inc_pid_controller_create(PID_PARAM_KP, PID_PARAM_KI, PID_PARAM_KD, PID_SAMPLE_TIME);
	
	// 1.4 Add two wheels
    c_wheels[0] = roc_robot.left_forward_wheel = wheel_create((motor_t)roc_robot.left_forward_motor, (encoder_t)roc_robot.left_forward_encoder, (controller_t)roc_robot.left_forward_pid, WHEEL_RADIUS, GEAR_RATIO);
    c_wheels[1] = roc_robot.right_forward_wheel = wheel_create((motor_t)roc_robot.right_forward_motor, (encoder_t)roc_robot.right_forward_encoder, (controller_t)roc_robot.right_forward_pid, WHEEL_RADIUS, GEAR_RATIO);

    // 2. Iinialize Kinematics - Two Wheel Differential Drive
    kinematics_t c_kinematics = kinematics_create(TWO_WD, WHEEL_DIST_X, WHEEL_DIST_Y, WHEEL_RADIUS);

    // 3. Initialize Chassis
    chas = chassis_create(c_wheels, c_kinematics);
	
	// 4. Enable Chassis
    chassis_enable(chas);
	
	rt_thread_t tid_car = RT_NULL;

    // thread
    tid_car = rt_thread_create("tcar",
                              car_thread, RT_NULL,
                              512,
                              15, 10);

    if (tid_car != RT_NULL)
    {
        rt_thread_startup(tid_car);
    }

	return RT_EOK;
}
MSH_CMD_EXPORT(robot_init, robot_init);

int roc_robot_go_forward(void)
{
    LOG_D("roc_robot_go_forward");
	
	chassis_set_velocity_x(chas, 1);

    roc_robot.status = E_FORWARD;
	
	return RT_EOK;
}
MSH_CMD_EXPORT(roc_robot_go_forward, roc_robot_go_forward);

int roc_robot_go_backward(void)
{
    LOG_D("roc_robot_go_backward");
	
	chassis_set_velocity_x(chas, -1);

    roc_robot.status = E_BACK;
	
	return RT_EOK;
}
MSH_CMD_EXPORT(roc_robot_go_backward, roc_robot_go_backward);

void car_thread(void *param)
{
    // TODO

    struct velocity target_velocity;

    target_velocity.linear_x = 0.00f;
    target_velocity.linear_y = 0;
    target_velocity.angular_z = 0;
    chassis_set_velocity(chas, target_velocity);

    // Open loop control
    // controller_disable(chas->c_wheels[0]->w_controller);
    // controller_disable(chas->c_wheels[1]->w_controller);

    while (1)
    {
        rt_thread_mdelay(50);
        chassis_update(chas);
    }
}
