#ifndef MAG_H
#define MAG_H

#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include "limits.h"

#include "types.h"

class magReadings {
    
    private:
        Adafruit_HMC5883_Unified magSensor = Adafruit_HMC5883_Unified(12345);
        vec_t *magData;

    public:
    
        magReadings(vec_t *magData);
        bool initializeMag();
        vec_t getMag();
};

#endif