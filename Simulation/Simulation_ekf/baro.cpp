#include "baro.h"

/*
TODO:
- Change environment values for Milan
*/

// Assumed environmental values:
float referencePressure = 1018;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 20;          // °C  measured local outdoor temp.
float barometerAltitude = 121;   // meters ... map readings + barometer position

// Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off, spi off
BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_16,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76);

BME280I2C bme(settings);

void initializeBarometer() {
    Wire.begin();
    while (!bme.begin()) {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch (bme.chipModel()) {
    case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
    case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
    default:
        Serial.println("Found UNKNOWN sensor! Error!");
    }
}

void getBarometer(bar_t *data) {
    float temp(NAN), hum(NAN), pres(NAN);
    unsigned long currentTime = micros();

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);

    EnvironmentCalculations::AltitudeUnit envAltUnit = EnvironmentCalculations::AltitudeUnit_Meters;
    EnvironmentCalculations::TempUnit envTempUnit = EnvironmentCalculations::TempUnit_Celsius;

    bme.read(pres, temp, hum, tempUnit, presUnit);
    float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
    data->pressure = pres;
    data->temperature = temp;
    data->altitude = altitude;
    data->dt = (currentTime >= data->t) ? (currentTime - data->t) / 1000.0f : (currentTime + (ULONG_MAX - data->t + 1)) / 1000.0f;
    data->t = currentTime;
}