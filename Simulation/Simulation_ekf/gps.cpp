#include "gps.h"

static const uint32_t GPSBaud = 115200;

// The TinyGPSPlus object
TinyGPSPlus gps;



static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsPort.available())
      gps.encode(gpsPort.read());
  } while (millis() - start < ms);
}

void initializeGPS(int gpsBaud, gps_t *coord) {
    // GPS serial init
    //gpsPort.begin(gpsBaud);
    byte gpsBaudConfig[] = { // 115200
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00,
    0xC2, 0x01, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x5E};
    byte gpsRateConfig[] = { // 50ms
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x32, 0x00, 0x01, 0x00, 0x01, 0x00, 0x48, 0xE6};
    byte gpsSaveConfig[] = {
    // Save Config
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
    0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xAF};

    // GPS config
    
    gpsPort.begin(9600);
    gpsPort.write(gpsBaudConfig, sizeof(gpsBaudConfig));
    gpsPort.write(gpsRateConfig, sizeof(gpsRateConfig));
    gpsPort.write(gpsSaveConfig, sizeof(gpsSaveConfig));
    gpsPort.end();

    gpsPort.begin(GPSBaud);
    

    // Set the initial position
    if (coord != NULL) {
        unsigned long startTime = millis();
        Serial.println(F("Feeding GPS, waiting for starting poisition"));
        while (gps.location.isUpdated() == 0) {
            //feedGPS();
            
            if (millis() - startTime > 10000) {
                Serial.println(F("GPS not found, using (0, 0) as starting position"));
                coord->lat = 0;
                coord->lon = 0;
                coord->alt = 0;
                coord->t = millis();
                coord->dt = 0;
                break;
            }
            Serial.println(gps.satellites.value());
            smartDelay(1000);
            Serial.println("finito Smartdelay");
            
        }
        // Store starting position
        Serial.println("iniziando Get");

        getGPS(coord, NULL);
        Serial.println("finito Get");
    }
}

bool isGPSUpdated() {
    return gps.location.isUpdated() || gps.speed.isUpdated() || gps.course.isUpdated();
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
        speed->dt = gps.speed.age();  // In milliseconds
    }
}

// To be ran frequently
void feedGPS() {
    while (gpsPort.available() > 0) {
        gps.encode(gpsPort.read());
    }
}