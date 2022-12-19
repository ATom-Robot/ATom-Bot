#include <rtthread.h>
#include <board.h>

#include "robot.h"
#include "diff_controller.h"

#define DBG_SECTION_NAME  "robot"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#if ENABLE_DBG
#define AUTO_STOP_INTERVAL 5000
static int lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

static void car_thread(void *param);

int robot_init(void)
{
    MX_TIM4_Init();

    robot.left_forward_motor   = dual_pwm_motor_create((char *)MOTOR_PWM, LEFT_FORWARD_PWM_CHANNEL, (char *)MOTOR_PWM, LEFT_BACKWARD_PWM_CHANNEL);
    robot.right_forward_motor  = dual_pwm_motor_create((char *)MOTOR_PWM, RIGHT_FORWARD_PWM_CHANNEL, (char *)MOTOR_PWM, RIGHT_BACKWARD_PWM_CHANNEL);

    robot.left_forward_encoder = ab_phase_encoder_create(LEFT_FORWARD_ENCODER_PIN_A, LEFT_FORWARD_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);
    robot.right_forward_encoder = ab_phase_encoder_create(RIGHT_FORWARD_ENCODER_PIN_A, RIGHT_FORWARD_ENCODER_PIN_B, PULSE_PER_REVOL, ENCODER_SAMPLE_TIME);

    // Enable PWM for motor
    motor_enable((motor_t)robot.left_forward_motor);
    motor_enable((motor_t)robot.right_forward_motor);

    // Enable Encoder's interrupt
    encoder_enable((encoder_t)robot.left_forward_encoder);
    encoder_enable((encoder_t)robot.right_forward_encoder);

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

#if ENABLE_DBG
static int motor_test(int arvc, char **argv)
{
    if (arvc < 2)
        return RT_ERROR;

    int arg1 = atoi(argv[1]);
    int arg2 = atoi(argv[2]);

    LOG_D("set speed:%d %d[m/s]", arg1, arg2);

    lastMotorCommand = rt_tick_get();
    if (arg1 == 0 && arg2 == 0)
    {
        motor_stop((motor_t)robot.left_forward_motor);
        motor_stop((motor_t)robot.right_forward_motor);

        resetPID();
        moving = 0;

        LOG_D("resetPID");
    }
    else
    {
      moving = 1;
    }

    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;

    return RT_EOK;
}
MSH_CMD_EXPORT(motor_test, roc_robot_go_forward);
#endif

static void car_thread(void *param)
{
#if ENABLE_DBG
    ano_init((char *)"uart3");
#endif
	
	InitPID();
    resetPID();

    while (1)
    {
        rt_thread_mdelay(50);

        updatePID();

#if ENABLE_DBG
        // Check to see if we have exceeded the auto-stop interval
        if ((rt_tick_get() - lastMotorCommand) > AUTO_STOP_INTERVAL)
        {
            motor_stop((motor_t)robot.left_forward_motor);
            motor_stop((motor_t)robot.right_forward_motor);
            moving = 0;
        }
    }
#endif
}
