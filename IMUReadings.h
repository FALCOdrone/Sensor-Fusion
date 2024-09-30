#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

#include "limits.h"
#include "pinDef.h"
#include "types.h"

class IMUReadings {

    private:

        MPU6050 mpu;
        bool dmpReady = false;   // set true if DMP init was successful
        uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
        uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;      // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64];  // FIFO storage buffer
        vec_t *gyro;
        quat_t *quat;
        attitude_t *att;
        vec_t *accel;

    public:

        IMUReadings(vec_t *gyro, quat_t *quat, attitude_t *att, vec_t *accel);
        void initializeImu();
        quat_t getQuaternion();
        attitude_t getAttitude();
        vec_t *getRawAccel();
        vec_t *getRawGyro();
        vec_t *getRealAccel();
        vec_t *getWorldAccel();
        vec_t getAcceleration();
        vec_t getGyro();
};

#endif