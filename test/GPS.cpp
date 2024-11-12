#include "GPS.h"

// The TinyGPSPlus object
TinyGPSPlus gps;

GPS::GPS(gps_t *coord, vec_t *speed) {
    this->coord = coord;
    this->speed = speed;
}

void GPS::smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (gpsPort.available())
            gps.encode(gpsPort.read());
    } while (millis() - start < ms);
}

bool GPS::initialize() {
    int gpsBaud = 115200;
    coord = NULL;
    // GPS serial init
    // gpsPort.begin(gpsBaud);
    byte gpsBaudConfig[] = {// 115200
                            0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00,
                            0xC2, 0x01, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x5E};
    byte gpsRateConfig[] = {// 50ms
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
    if (coord != NULL)
    {
        unsigned long startTime = millis();
        Serial.println(F("Feeding GPS, waiting for starting poisition"));
        while (gps.location.isUpdated() == 0)
        {
            // feedGPS();

            if (millis() - startTime > 10000)
            {
                Serial.println(F("GPS not found, using (0, 0) as starting position"));
                coord->lat = 0;
                coord->lon = 0;
                coord->alt = 0;
                coord->t = millis();
                coord->dt = 0;
                return false;
            }
            smartDelay(1000);
        }
        Serial.println("GPS found, calculating variance");

        gps_t gpsData[5];
        int k = 0;

        int numSamples = 5;

        gps_t var;
        gps_t mean;

        while (k < numSamples || var.lat > MAX_VAR_DEG_LAT || var.lon > MAX_VAR_DEG_LONG /*|| var.alt > MAX_VAR_ALT*/)
        {

            while (!getGPS(&gpsData[k % numSamples], NULL))
            {
                feedGPS();
            }

            if (k >= numSamples - 1)
            {
                var = {0.0, 0.0, 0.0, long(0.0), 0.0};
                mean = {0.0, 0.0, 0.0, long(0.0), 0.0};

                for (int i = 0; i < numSamples; i++)
                {
                    mean.lat += gpsData[i].lat;
                    mean.lon += gpsData[i].lon;
                    mean.alt += gpsData[i].alt;
                }
                mean.lat /= numSamples;
                mean.lon /= numSamples;
                mean.alt /= numSamples;

                for (int i = 0; i < numSamples; i++)
                {
                    var.lat += pow(gpsData[i].lat - mean.lat, 2);
                    var.lon += pow(gpsData[i].lon - mean.lon, 2);
                    var.alt += pow(gpsData[i].alt - mean.alt, 2);
                }
                var.lat /= numSamples - 1;
                var.lon /= numSamples - 1;
                var.alt /= numSamples - 1;

                Serial.printf("varLat: %0.12f, varLon: %0.12f, varAlt: %0.12f\n", var.lat, var.lon, var.alt);
            }
            smartDelay(1000);
            k++;
        }
        
        Serial.printf("Variance calculated, setting initial position to %0.6f, %0.6f, %0.6f\n", mean.lat, mean.lon, mean.alt);
        coord->lat = mean.lat;
        coord->lon = mean.lon;
        coord->alt = mean.alt;
        coord->t = millis();
        coord->dt = 0;
    }
    return true;
}

bool GPS::isGPSUpdated() {
    return gps.location.isUpdated() || gps.speed.isUpdated() || gps.course.isUpdated();
}
        
bool GPS::getGPS(gps_t *gpsCoord, vec_t *speed) {
    bool update_location = gps.location.isUpdated();

    if (update_location)
    {
        gpsCoord->lat = gps.location.lat();
        gpsCoord->lon = gps.location.lng();
        gpsCoord->alt = gps.altitude.meters();
        gpsCoord->t = gps.time.value();
        gpsCoord->dt = gps.time.age() * 1000.0; // In microseconds
        // Serial.println("location is updated");
        /*Serial.println("lat, lng, alt");
        Serial.println(gps.location.lat(), 6);
        Serial.println(gps.location.lng(), 6);
        Serial.println(gps.altitude.meters(), 6);
        Serial.println(gps.hdop.hdop());*/
    }

    bool update_speed = gps.speed.isUpdated();

    if (update_speed && speed != NULL)
    {
        // Convert course and speed to x and y components
        speed->x = gps.speed.mps() * cos(gps.course.deg() * DEG_TO_RAD);
        speed->y = gps.speed.mps() * sin(gps.course.deg() * DEG_TO_RAD);
        speed->dt = gps.speed.age(); // In milliseconds
    }
    return update_location || update_speed;
}

// To be ran frequently
void GPS::feedGPS()
{
    while (gpsPort.available() > 0)
    {
        gps.encode(gpsPort.read());
    }
}

vec_t GPS::getPos(float lat0, float lon0, float alt0) {
    vec_t posGPS;

    if (getGPS(gpsCoord, speed))
    {
        posGPS.x = 111320 * (gpsCoord->lat - lat0);                         // north
        posGPS.y = 111320 * cos(lat0) * (gpsCoord->lon - lon0);             // east
        posGPS.z = gpsCoord->alt - alt0;                                    // down*/
        posGPS.dt = gpsCoord->dt;
    }
    return posGPS;
}