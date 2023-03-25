#include "HAL/HAL.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "MPU6050.h"

static MPU6050 mpu;

static bool dmpReady = false;
static uint8_t fifoBuffer[64];
// orientation/motion vars
static Quaternion q;        // [w, x, y, z]         quaternion container
static VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];             // [psi, theta, phi]    Euler angle container
float ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t roll, yaw, pitch;

void HAL::IMU_Init()
{
    uint16_t packetSize;

    Wire.setClock(400000);
    mpu.initialize();
    if (mpu.testConnection())
    {
        Serial.println("MPU connection failed.");
    }

    Serial.println(F("Initializing DMP..."));
    int devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);

    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void HAL::IMU_Update()
{
    static IMU_Info_t imuInfo;

    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        imuInfo.yaw = ypr[0] * 180 / M_PI;
        imuInfo.pitch = ypr[1] * 180 / M_PI;
        imuInfo.roll = ypr[2] * 180 / M_PI;

        // Serial.println(imuInfo.yaw);
    }
}
