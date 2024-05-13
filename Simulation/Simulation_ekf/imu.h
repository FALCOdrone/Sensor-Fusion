#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

#include "limits.h"
#include "pinDef.h"
#include "types.h"

// #define ULONG_MAX 0xFFFFFFFF

void initializeImu();
void getQuaternion(quat_t *quat);
void getAttitude(attitude_t *ypr);
void getRawAccel(vec_t *accel);
void getRawGyro(vec_t *gyro);
void getRealAccel(vec_t *accel);
void getWorldAccel(vec_t *accel);
void getAcceleration(vec_t *accel);
void getGyro(vec_t *gyro);

void printIMUData(vec_t *data, const char *unit);
void printIMUData(quat_t *quat);
void printIMUData(attitude_t *att);

void logIMU(vec_t *pos, vec_t *speed, vec_t *accel);

#endif