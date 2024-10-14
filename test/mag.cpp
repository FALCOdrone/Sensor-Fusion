#include "mag.h"

Magnetometer::Magnetometer(vec_t *magData) {
    this->magData = magData;
}

bool Magnetometer::initialize() {
    if (!magSensor.begin()) {
        Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
        return false;
    }
    return true;
}

vec_t Magnetometer::getMag() {
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

void Magnetometer::getPairData(vec_t pairMagData[2]) {
    if (micros() - pairMagData[0].t > 5000) {
        pairMagData[1] = getMag();
    }
}
