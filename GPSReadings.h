#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "types.h"
#include "pinDef.h"

#endif

class GPSReadings
{
    private:
    
        #define MAX_VAR_DEG_LAT pow((2 / 111320.0),2) //2m variance
        #define MAX_VAR_DEG_LONG pow(2 / (111320.0 * cos(45.4778828 / 180 * PI)),2) // latitiude set in front of building 6
        #define MAX_VAR_ALT 0.2

        static const uint32_t GPSBaud = 115200;

        unsigned long ms;
        int gpsBaud;
        gps_t *coord;
        gps_t *gpsCoord;
        vec_t *speed;

    public:

        GPSReadings(gps_t *coord, vec_t *speed);
        void smartDelay(unsigned long ms);
        bool initializeGPS();
        bool isGPSUpdated();
        bool getGPS(gps_t *gpsCoord, vec_t *speed);
        void feedGPS();

};