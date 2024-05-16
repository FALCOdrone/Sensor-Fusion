#include "mag.h"

Adafruit_HMC5883_Unified magSensor = Adafruit_HMC5883_Unified(12345);

void initializeMag() {
    if (!magSensor.begin()) {
        Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    }
}

void getMag(vec_t *magData) {
    sensors_event_t event;
    unsigned long currentTime = micros();
    magSensor.getEvent(&event);
    magData->x = event.magnetic.x;  // in micro-Teslas
    magData->y = event.magnetic.y;
    magData->z = event.magnetic.z;
    magData->dt = (currentTime >= magData->t) ? (currentTime - magData->t) / 1000.0f : (currentTime + (ULONG_MAX - magData->t + 1)) / 1000.0f;
    magData->t = currentTime;
}