#include "gps.h"

static const uint32_t GPSBaud = 115200;

// The TinyGPSPlus object
TinyGPSPlus gps;
#define gpsPort Serial2
byte gpsBaudConfig[] = {  // 115200
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00,
    0xC2, 0x01, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x5E};
byte gpsRateConfig[] = {  // 50ms
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x32, 0x00, 0x01, 0x00, 0x01, 0x00, 0x48, 0xE6};
byte gpsSaveConfig[] = {
    // Save Config
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
    0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xAF};

void initializeGPS(int gpsBaud) {
    // GPS serial init
    gpsPort.begin(gpsBaud);
    // GPS config
    /*
    gpsPort.write(gpsBaudConfig, sizeof(gpsBaudConfig));
    //gpsPort.write(gpsRateConfig, sizeof(gpsRateConfig));
    gpsPort.write(gpsSaveConfig, sizeof(gpsSaveConfig));
    gpsPort.end();
    gpsPort.begin(115200);
    */
}

void getGPS(gps_t *gpsCoord, vec_t *speed) {
    if (gps.location.isUpdated()) { 
        gpsCoord->lat = gps.location.lat();
        gpsCoord->lon = gps.location.lng();
        gpsCoord->alt = gps.altitude.meters();
        gpsCoord->t = gps.time.value();
        gpsCoord->dt = gps.time.age() * 1000.0;  // In microseconds
    }

    if (gps.speed.isUpdated() && gps.course.isUpdated()) {
        // Convert course and speed to x and y components
        speed->x = gps.speed.mps() * cos(gps.course.deg() * DEG_TO_RAD);
        speed->y = gps.speed.mps() * sin(gps.course.deg() * DEG_TO_RAD);
        speed->dt = gps.speed.age() * 1000.0;  // In microseconds
    }
}

// To be ran frequently
void feedGPS() {
    while (gpsPort.available() > 0) {
        gps.encode(gpsPort.read());
    }
}