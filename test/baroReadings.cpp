#include "baroReadings.h"

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

Barometer::Barometer(bar_t *bar) {
    this->data = bar;
}
        
bool Barometer::initialize() {

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
    
    int numSamples = 5;
    bar_t bar[numSamples];
    double var;
    double mean;
    int k = 0;
    double prev = micros();
    
    while (k < numSamples || var > MAX_VAR_BARO )
    {
        bar[k%numSamples] = getBarometer();

        double dt = micros() - prev;
        Serial.printf("%d, %0.12f: %0.12f\n", k, dt, bar[k%numSamples].altitude);
        prev = micros();
        if (k >= numSamples-1)
        {
            var = 0.0;
            mean = 0.0;

            for (int i = 0; i < numSamples; i++)
            {
                mean += bar[i].altitude;
            }
            mean /= numSamples;
            

            for (int i = 0; i < numSamples; i++)
            {
                var += pow(bar[i].altitude - mean, 2);
            }
            var /= numSamples;

            Serial.printf("varAlt: %0.12f, mean: %0.12f\n", var, mean);
        }
        
        k++;
    }
    bar0 = mean;
    
    return true;
}

bar_t Barometer::getBarometer() {
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
    data->altitude = altitude - bar0;
    data->dt = (currentTime >= data->t) ? (currentTime - data->t) / 1000.0f : (currentTime + (ULONG_MAX - data->t + 1)) / 1000.0f;
    data->t = currentTime;

    return *data;
}
