#include <Arduino.h>

#include "QuadEstimatorEKF.h"
#include "gps.h"
#include "imu.h"
#include "baro.h"
#include "mag.h"
#include "motor.h"
#include "pinDef.h"
#include "radio.h"
#include "utils.h"

#define DEBUG 1
#define DEBUG_GPS 0

// variables
vec_t accelWithOffset;
VectorXf accelWithOffset2(3);
vec_t gyro;
VectorXf fixed_accel(3);
vec_t posGPS;
vec_t posGPS0;  // position (from gps) of starting point
gps_t coordGPS;
vec_t speedGPS;
vec_t mag;
vec_t pos;
vec_t speed;
quat_t quat;
attitude_t att;
bar_t bar;
float yawMag;
float lat0;         // latitude at starting point, used for projection (lat long -> x y)
float r = 6371000;  // earth radius (m)

// Parameters for EKF
const int Nstate = 7;
VectorXf ini_state(Nstate);
VectorXf ini_stdDevs(Nstate);
VectorXf predict_state(Nstate);
MatrixXf R(3, 3);

// timing parameters
unsigned long currentTime, prevTime;
int GPSrate = 1;

// initialization of the constructor for estimation
QuadEstimatorEKF estimation;

void setup() {
    Serial.begin(115200);
    Serial.println("Initialization starting");

    initializeImu();
    initializeGPS(115200, &coordGPS);
    initializeMag();
    initializeBarometer();
    // initializeMotors();
    // initializeRadio();

    ini_state.setZero();
    ini_stdDevs.setOnes();
    estimation.initialize(ini_state, ini_stdDevs);

    // setting initial values for estimation parameters/variables
    R << cos(PI / 4), sin(PI / 4), 0,
        -sin(PI / 4), cos(PI / 4), 0,
        0, 0, 1;
    lat0 = coordGPS.lat;
    posGPS0.x = r * coordGPS.lat;              // north
    posGPS0.y = r * coordGPS.lon * cos(lat0);  // east
    posGPS0.z = coordGPS.alt;                  // up

    currentTime = micros();
    Serial.println("Initialization done");
}

void loop() {
    prevTime = currentTime;
    currentTime = micros();

    // getting values from imu
    getAcceleration(&accelWithOffset);
    getGyro(&gyro);
    if(DEBUG){
      Serial.print("Acc");
      printIMUData(&accelWithOffset, "m/s^2");
      Serial.print("Gyro");
      printIMUData(&gyro, "rad/s");
    }
    
    getGPS(&coordGPS, &speedGPS); 
    if(isGPSUpdated()){
      posGPS.x = r*coordGPS.lat - posGPS0.x;  //north
      posGPS.y = r*coordGPS.lon*cos(lat0) - posGPS0.y;  //east
      posGPS.z = -coordGPS.alt + posGPS0.z;  //down
      posGPS.dt = coordGPS.dt;
      if(DEBUG_GPS || DEBUG){
        Serial.print("GPS pos");
        printIMUData(&posGPS, "m");
        Serial.print("GPS speed");
        printIMUData(&speedGPS, "m/s");
      }
    }

    getMag(&mag);
    if(DEBUG){
      Serial.print("Mag");
      printIMUData(&mag, "µT");
    }
    
    getBarometer(&bar);
    if(DEBUG){
      Serial.print("Bar");
      //printIMUData(&bar, "m");
      Serial.print("MISSING FUNCTION TO PRINT BAROMETER DATA");
    }
    // removing the angular offset
    accelWithOffset2(0) = accelWithOffset.x;
    accelWithOffset2(1) = accelWithOffset.y;
    accelWithOffset2(2) = accelWithOffset.z;

    fixed_accel = R * accelWithOffset2;  // body frame accelleration without offset
    yawMag = estimation.yawFromMag(mag, quat);
    
    
    // EKF estimation for attitude, speed and position
    estimation.kf_attitudeEstimation(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // quaternion attitude estimation
    estimation.getAttitude(&quat, &att);
    estimation.predict(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // prediction of the (x, y, z) position and velocity

    // compute the update from gps
    if (isGPSUpdated()) {
        estimation.updateFromGps(Vector3f(posGPS.x, posGPS.y, posGPS.z), Vector3f(speedGPS.x, speedGPS.y, speedGPS.z), posGPS.dt);
    }
    estimation.updateFromMag(yawMag, mag.dt);  
    estimation.updateFromBar(bar.pressure, bar.dt);

    estimation.getPosVel(&pos, &speed);
    /*
    Serial.print("Quaternion:");
    Serial.print(quat.w);
    Serial.print(",");
    Serial.print(quat.x);
    Serial.print(",");
    Serial.print(quat.y);
    Serial.print(",");
    Serial.println(quat.z);
    */
    if(DEBUG){
      Serial.print("EKF Quat");
      printIMUData(&quat);
      //Serial.print("EKF Att");
      //printIMUData(&att);
      /*Serial.print("EKF Pos");
      printIMUData(&pos, "m");
      Serial.print("EKF Speed");
      printIMUData(&speed, "m/s");*/
    }
    feedGPS();
    loopRate(2000);
}

void loopRate(int freq) {
    // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
     * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
     * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
     * and remain above 2kHz, without needing to retune all of our filtering parameters.
     */
    float invFreq = 1.0 / freq * 1000000.0;
    unsigned long checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - currentTime)) {
        feedGPS();
        checker = micros();
    }
}
