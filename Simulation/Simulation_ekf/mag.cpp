#include "mag.h"

Adafruit_HMC5883_Unified magSensor = Adafruit_HMC5883_Unified(12345);

void initializeMag() {
    if (!magSensor.begin()) {
        Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
        while (1)
            ;
    }
}

void getMag(vec_t *magData) {
    sensors_event_t event;
    magSensor.getEvent(&event);
    magData->x = event.magnetic.x;  // in micro-Teslas
    magData->y = event.magnetic.y;
    magData->z = event.magnetic.z;
    magData->dt = (event.timestamp * 1000.0 >= magData->t) ? (event.timestamp * 1000.0 - magData->t) : (magData->t + (ULONG_MAX - magData->t + 1));
    magData->t = event.timestamp * 1000.0;
}