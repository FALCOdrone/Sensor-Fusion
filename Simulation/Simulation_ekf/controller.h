#ifndef CTRL_H
#define CTRL_H

#include "types.h"
#include <Arduino.h>

void controlMixer(float throttleDes, float mCmdScaled[], attitude_t attPID);
void controlANGLE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, PID_t *PID);
void controlANGLE2(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, attitude_t *attIMUprev, PID_t *PIDol, PID_t *PIDil);
void controlRATE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, vec_t *prevGyFro, attitude_t attIMU, PID_t *PID);

#endif