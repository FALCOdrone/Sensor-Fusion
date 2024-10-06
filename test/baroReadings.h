#ifndef BAR_H
#define BAR_H

#include <Arduino.h>

#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>

#include "limits.h"
#include "types.h"

class Barometer {

    private:

        #define MAX_VAR_BARO pow(1.0,2) // meters^2
        // Assumed environmental values:
        float referencePressure = 1018;  // hPa local QFF (official meteor-station reading)
        float outdoorTemp = 20;          // °C  measured local outdoor temp.
        float bar0 = 0;   // meters ... map readings + barometer position
        bar_t *data;

    public:

        Barometer(bar_t *bar);
        bool initialize();
        bar_t getBarometer();

};

#endif