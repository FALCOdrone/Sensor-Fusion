#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "types.h"

void setupBlink(int numBlinks, int upTime, int downTime);

void lpFilter(vec_t *curr, vec_t *prev, float b);
void lpFilter(attitude_t *curr, attitude_t *prev, float b);
void lpFilter(quat_t *curr, quat_t *prev, float b);

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq);

void printData(vec_t *data, const char *unit = "");
void printData(quat_t *quat);
void printData(attitude_t *att);
void printData(gps_t *gps);
void printData(bar_t *bar);
void doReboot();

#endif