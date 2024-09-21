#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "types.h"
#include "pinDef.h"

bool initializeGPS(int baudRate = 115200, gps_t *coord = NULL);
bool getGPS(gps_t *gpsCoord, vec_t *speed);
bool isGPSUpdated();
void feedGPS();
void smartDelay(unsigned long ms);

#endif