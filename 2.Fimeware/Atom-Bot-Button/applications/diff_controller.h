/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "robot.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/* PID setpoint info For a Motor */
typedef struct
{
    double TargetTicksPerFrame;    // target speed in ticks per frame
    long Encoder;                  // encoder count
    long PrevEnc;                  // last encoder count

    /*
    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */
    int PrevInput;                  // last input
    //int PrevErr;                  // last error

    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //int Ierror;
    int ITerm;                      //integrated term

    long output;                    // last motor setting

    int Kp, Kd, Ki, Ko;
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

unsigned char moving = 0; // is the base in motion?

void InitPID(void)
{
    /* PID Parameters */
    leftPID.Kp = 288;
    leftPID.Kd = 380;
    leftPID.Ki = 0;
    leftPID.Ko = 100;

    rightPID.Kp = 283;
    rightPID.Kd = 330;
    rightPID.Ki = 0;
    rightPID.Ko = 100;
}
/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(void)
{
    leftPID.TargetTicksPerFrame = 0.0;
    leftPID.Encoder = encoder_read((encoder_t)robot.left_forward_encoder);
    leftPID.PrevEnc = leftPID.Encoder;
    leftPID.output = 0;
    leftPID.PrevInput = 0;
    leftPID.ITerm = 0;

    rightPID.TargetTicksPerFrame = 0.0;
    rightPID.Encoder = encoder_read((encoder_t)robot.right_forward_encoder);
    rightPID.PrevEnc = rightPID.Encoder;
    rightPID.output = 0;
    rightPID.PrevInput = 0;
    rightPID.ITerm = 0;
}

#if ENABLE_DBG
    static int inputL;
    static int inputR;
#endif
/* PID routine to compute the next motor commands */
rt_inline void doPID(SetPointInfo *p)
{
    long Perror;
    long output;
    int input;

    //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
    input = p->Encoder - p->PrevEnc;
    Perror = p->TargetTicksPerFrame - input;

#if ENABLE_DBG
    if (p == &leftPID)
    {
        inputL = input;
    }
    if (p == &rightPID)
    {
        inputR = input;
    }

    ano_send_user_data(1, (rt_int16_t)inputL,
                       (rt_int16_t)inputR,
                       (rt_int16_t)0,
                       (rt_int16_t)0);
#endif
    /*
    * Avoid derivative kick and allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
    // p->PrevErr = Perror;
    output = (p->Kp * Perror - p->Kd * (input - p->PrevInput) + p->ITerm) / p->Ko;
    p->PrevEnc = p->Encoder;

    output += p->output;
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    constrain(output, MIN_PWM, MAX_PWM);

    /*
    * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    p->ITerm += p->Ki * Perror;

    p->output = output;
    p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
    /* Read the encoders */
    leftPID.Encoder = encoder_read((encoder_t)robot.left_forward_encoder);
    rightPID.Encoder = encoder_read((encoder_t)robot.right_forward_encoder);

    /* If we're not moving there is nothing more to do */
    if (!moving)
    {
        /*
        * Reset PIDs once, to prevent startup spikes,
        * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
        * PrevInput is considered a good proxy to detect
        * whether reset has already happened
        */
        if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
        return;
    }

    /* Compute PID update for each motor */
    doPID(&leftPID);
    doPID(&rightPID);

    /* Set the motor speeds accordingly */
    motor_run_double((motor_t)robot.left_forward_motor, (motor_t)robot.right_forward_motor, leftPID.output, rightPID.output);
}

