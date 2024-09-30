#include "magReadings.h"

magReadings::magReadings(vec_t *magData) {
    this->magData = magData;
}

bool magReadings::initializeMag() {
    if (!magSensor.begin()) {
        Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
        return false;
    }
    return true;
}

vec_t magReadings::getMag() {
    sensors_event_t event;
    unsigned long currentTime = micros();
    magSensor.getEvent(&event);
    magData->x = event.magnetic.x;  // in micro-Teslas
    magData->y = event.magnetic.y;
    magData->z = event.magnetic.z;
    magData->dt = (currentTime >= magData->t) ? (currentTime - magData->t) / 1000.0f : (currentTime + (ULONG_MAX - magData->t + 1)) / 1000.0f;
    magData->t = currentTime;

    return *magData;
}

