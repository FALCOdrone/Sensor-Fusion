#ifndef BAR_H
#define BAR_H

#include <Arduino.h>

#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>

#include "limits.h"
#include "types.h"

bool initializeBarometer();
void getBarometer(bar_t *baroData);

#endif