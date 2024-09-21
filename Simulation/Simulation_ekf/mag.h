#ifndef MAG_H
#define MAG_H

#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include "limits.h"

#include "types.h"

bool initializeMag();
void getMag(vec_t *magData);

#endif