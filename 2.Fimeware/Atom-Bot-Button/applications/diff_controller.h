/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "robot.h"
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
    int PrevInput;					// last input
    //int PrevErr;					// last error

    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    //int Ierror;
    int ITerm;						//integrated term

    long output;					// last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 23;
int Kd = 2;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID()
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
void doPID(SetPointInfo *p)
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
    output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
    p->PrevEnc = p->Encoder;

    output += p->output;
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
        output = MAX_PWM;
    else if (output <= -MAX_PWM)
        output = -MAX_PWM;
    else
        /*
        * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
        */
        p->ITerm += Ki * Perror;

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

