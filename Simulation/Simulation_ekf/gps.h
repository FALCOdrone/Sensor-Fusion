#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "types.h"

void initializeGPS(int baudRate = 115200, gps_t *coord = NULL);
void getGPS(gps_t *gpsCoord, vec_t *speed);
bool isGPSUpdated();
void feedGPS();

#endif