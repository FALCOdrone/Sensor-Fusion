#ifndef IMU_H
#define IMU_H

#include <Wire.h>

#include "FastIMU.h"
//#include "Madgwick.h"
#include "pinDef.h"
#include "types.h"
#include "limits.h"

//#define ULONG_MAX 0xFFFFFFFF

void initializeImu(int calibrate = 1);
//void getAttitude(quat_t *quat, attitude_t *attitude);
void getAcceleration(vec_t *accel);
void getGyro(vec_t *gyro);

void printIMUData(vec_t *data, const char *unit);
void printIMUData(quat_t *quat);

void logIMU(vec_t *pos, vec_t *speed, vec_t *accel);

#endif