#include <Arduino.h>

#include "EstimatorEKF.h"
#include "baro.h"
#include "GPS.h"
#include "IMU.h"
#include "mag.h"
#include "common\pinDef.h"
#include "common\utils.h"

#include "control\motor.h"
#include "control\radio.h"

#define DEBUG_ALL 1
#define DEBUG_GPS 0
#define DEBUG_MAGYAW 0
#define DEBUG_ACC 0
#define DEBUG_GYRO 0
#define DEBUG_MAG 0
#define DEBUG_BAR 0
#define DEBUG_POS 0
#define DEBUG_VEL 0
#define DEBUG_QUAT 0
#define DEBUG_YPR 0
#define DEBUG_SERIALCOMMANDS 0

// variables
vec_t accIMUFrame;
Vector3f accBodyFrame;
vec_t gyro;
vec_t posGPS;
vec_t posGPS0; // position (from gps) of starting point
gps_t coordGPS;
vec_t speedGPS;
vec_t mag;
vec_t pos;
vec_t speed;
vec_t MaxPosGpsValue; 
vec_t MaxVelGpsValue; 
vec_t prevGpsPos;
vec_t currGpsPos;
vec_t prevGpsVel;
vec_t currGpsVel;
vec_t prevMag;
float prevMagYaw;
float currMagYaw;
quat_t prevQuat;
quat_t currQuat;
quat_t quat;
attitude_t att;
bar_t bar;
bar_t MaxBarValue; // max tollerance values of the barometer, needs to be set
bar_t prevBar;
bar_t currBar;
float MaxMagValue; // max tollerance values of the magnetometer, needs to be set
float yawMag;
float lat0;        // latitude at starting point, used for projection (lat long -> x y)
float lon0;        
float alt0;
float r = 6371000; // earth radius (m)

bool validGPS;
bool validMag;
bool validBaro;

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
EKF estimation;

// initialization of the gps constructor for the gps readings
GPS gpsSensor(&coordGPS, &speedGPS);

// initialization of the mag constructor for the magnetometer readings
Magnetometer magSensor(&mag);

// initialization of the baro constructor for the barometer readings
Barometer baroSensor(&bar);

// initialization of the imu constructor for the imu readings
IMU imuSensor(&gyro, &quat, &att, &accIMUFrame);

Utils utils;


void setup()
{
    Serial.begin(115200);
    bool start = false;
    while(DEBUG_SERIALCOMMANDS && start == false) { //if DEBUG_SERIALCOMMANDS is 0, the program will start immediately
        // wait for bytes available to read
        while(!Serial.available()) {
            delay(10);
        }
        // Read the incoming byte
        char incomingChar = Serial.read();

        // If the byte is 's', start the program
        if (incomingChar == 's'){ 
            Serial.println("Initialization starting");
            start = true;
            SCB_AIRCR = 0x05FA0004; //to reboot the Teensy
        }
        else
            Serial.println("Initialization not started, waiting for 's' command");
    }
    imuSensor.initialize();
    validGPS = gpsSensor.initialize();
    validMag = magSensor.initialize();
    validBaro = baroSensor.initialize();
    // initialize();
    // initializeRadio();

    ini_state.setZero();
    ini_stdDevs.setOnes();
    estimation.initialize(ini_state, ini_stdDevs);

    // setting initial values for estimation parameters/variables
    R << cos(PI / 4), sin(PI / 4), 0,
        -sin(PI / 4), cos(PI / 4), 0,
        0, 0, 1;

    // Get first readings to fix the time for the first loop
    currentTime = micros();
    accIMUFrame.t = currentTime;
    gyro.t = currentTime;
    mag.t = currentTime;
    bar.t = currentTime;

    if (validGPS)
    {
        lat0 = coordGPS.lat;
        lon0 = coordGPS.lon;
        alt0 = coordGPS.alt;

        /*posGPS0.x = r * coordGPS.lat;             // north
        posGPS0.y = r * coordGPS.lon * cos(lat0); // east
        posGPS0.z = coordGPS.alt;                 // up*/
    }

    // setting the maximum values for the gps and mag readings
    MaxPosGpsValue.x = MaxPosGpsValue.y = MaxPosGpsValue.z = 20;  // in meters
    MaxVelGpsValue.x = MaxVelGpsValue.y = MaxVelGpsValue.z = 3;   // in m/s
    MaxMagValue = 10; // in degrees
    MaxBarValue.altitude = 10; // in meters

    // Get first gps reading to fix the time for the first loop
    prevGpsPos = gpsSensor.getPos(lat0, lon0, alt0);
    prevGpsVel = speedGPS;

    // Get first mag reading to fix the time for the first loop
    prevMag = magSensor.getMag();
    prevQuat = imuSensor.getQuaternion();

    // Get first bar reading to fix the time for the first loop
    prevBar = baroSensor.getBarometer();

    Serial.println("Initialization done");
}

void loop()
{
    prevTime = currentTime;
    currentTime = micros();

    // Getting values from imu
    if (micros() - accIMUFrame.t >= 5000)
    { // 200Hz
        accIMUFrame = imuSensor.getAcceleration();
        if (DEBUG_ACC || DEBUG_ALL)
        {
            Serial.print("Acc:\t");
            utils.printData(&accIMUFrame); 
        }
    }

    if (micros() - gyro.t >= 5000)
    { // 200Hz
        gyro = imuSensor.getGyro();
        if (DEBUG_GYRO || DEBUG_ALL)
        {
            Serial.print("Gyro:\t");
            utils.printData(&gyro);
        }
    }

    if (validGPS && gpsSensor.getGPS(&coordGPS, &speedGPS))
    {
        /*posGPS.x = r * coordGPS.lat - posGPS0.x;             // north
        posGPS.y = r * coordGPS.lon * cos(lat0) - posGPS0.y; // east
        posGPS.z = -coordGPS.alt + posGPS0.z;                // down
        posGPS.dt = coordGPS.dt; */

        posGPS = gpsSensor.getPos(lat0, lon0, alt0);
        currGpsPos = posGPS;
        currGpsVel = speedGPS;
        
        float deltaX = currGpsPos.x - prevGpsPos.x;
        float deltaY = currGpsPos.y - prevGpsPos.y;
        float deltaZ = currGpsPos.z - prevGpsPos.z;

        float deltaVelX = currGpsVel.x - prevGpsVel.x;
        float deltaVelY = currGpsVel.y - prevGpsVel.y;
        float deltaVelZ = currGpsVel.z - prevGpsVel.z;

        estimation.updateFromGps(Vector3f(prevGpsPos.x, prevGpsPos.y, prevGpsPos.z), Vector3f(prevGpsVel.x, prevGpsVel.y, prevGpsVel.z), prevGpsPos.dt / 1000.0f);

        if (abs(deltaX) < MaxPosGpsValue.x && abs(deltaY) < MaxPosGpsValue.y && abs(deltaZ) < MaxPosGpsValue.z
            && abs(deltaVelX) < MaxVelGpsValue.x && abs(deltaVelY) < MaxVelGpsValue.y && abs(deltaVelZ) < MaxVelGpsValue.z) {
            estimation.updateFromGps(Vector3f(currGpsPos.x, currGpsPos.y, currGpsPos.z), Vector3f(currGpsVel.x, currGpsVel.y, currGpsVel.z), currGpsPos.dt / 1000.0f);
        }

       //estimation.updateFromGps(Vector3f(posGPS.x, posGPS.y, posGPS.z), Vector3f(speedGPS.x, speedGPS.y, speedGPS.z), posGPS.dt / 1000.0f);
        prevGpsPos = currGpsPos;  // update previous gps position

        if (DEBUG_GPS || DEBUG_ALL)
        {
            Serial.print("GPS_Pos:\t");
            utils.printData(&posGPS);
            /*Serial.print("GPS_LAT:\t");
            Serial.print(coordGPS.lat, 6);
            Serial.print("\tGPS_LON:\t");
            Serial.println(coordGPS.lon, 6);*/
            Serial.print("GPS_Speed:\t");
            utils.printData(&speedGPS);
        }
    }

    if (validMag && micros() - mag.t > 5000)
    { // 200Hz
        mag = magSensor.getMag();
        prevMagYaw = estimation.yawFromMag(prevMag, prevQuat);
        currMagYaw = estimation.yawFromMag(mag, quat);

        float deltaYaw = currMagYaw - prevMagYaw;

        estimation.updateFromMag(prevMagYaw, prevMag.dt / 1000.0f);

        if (abs(deltaYaw) < MaxMagValue) {
            estimation.updateFromMag(currMagYaw, mag.dt / 1000.0f);
        }

        //yawMag = estimation.yawFromMag(mag, quat);
        //estimation.updateFromMag(yawMag, mag.dt / 1000.0f);
        prevMag = mag;  // update previous mag reading

        if (DEBUG_MAG || DEBUG_ALL)
        {
            Serial.print("Mag:\t");
            utils.printData(&mag);
        }
    }

    if (validBaro && micros() - bar.t > 5000)
    {  // 200Hz
        bar = baroSensor.getBarometer();
        currBar = bar;

        float deltaAlt = currBar.altitude - prevBar.altitude;

        estimation.updateFromBar(prevBar.altitude, prevBar.dt / 1000.0f);
        
        if (abs(deltaAlt) < MaxBarValue.altitude) {
            estimation.updateFromBar(currBar.altitude, currBar.dt / 1000.0f);
        }

        //estimation.updateFromBar(bar.altitude, bar.dt / 1000.0f);
        prevBar = currBar;  // update previous barometer reading

        if (DEBUG_BAR || DEBUG_ALL)
        {
            Serial.print("Bar:\t");
            utils.printData(&bar);
        }
    }

    // // removing the angular offset

    accBodyFrame = R * Vector3f(accIMUFrame.x, accIMUFrame.y, accIMUFrame.z); // acceleration in drone frame
    // // EKF estimation for attitude, speed and position
    // // estimation.kf_attitudeEstimation(accBodyFrame, Vector3f(gyro.x, gyro.y, gyro.z), accIMUFrame.dt);  // quaternion attitude estimation
    quat = imuSensor.getQuaternion();
    att = imuSensor.getAttitude();
    estimation.xt_at << quat.w, quat.x, quat.y, quat.z; // just copy quaternion from dmp
    estimation.estAttitude = estimation.EPEuler321(estimation.xt_at);
    estimation.predict(accBodyFrame, Vector3f(gyro.x, gyro.y, gyro.z), accIMUFrame.dt / 1000.0f); // prediction of the (x, y, z) position and velocity
    estimation.getPosVel(&pos, &speed);

    if (DEBUG_QUAT || DEBUG_ALL)
    {
        Serial.print("EKF_Quat:\t");
        utils.printData(&quat);
    }
    if (DEBUG_MAGYAW || DEBUG_ALL)
    {
        // Serial.print("YAW_mag:\t");
        // Serial.print(yawMag);   //da problemi con serialtomat.py
    }
    if (DEBUG_YPR || DEBUG_ALL)
    {
        Serial.print("EKF_YPR:\t");
        utils.printData(&att);
    }
    if (DEBUG_POS || DEBUG_ALL)
    {
        Serial.print("EKF_Pos:\t");
        utils.printData(&pos);
    }
    if (DEBUG_VEL || DEBUG_ALL)
    {
        Serial.print("EKF_Speed:\t");
        utils.printData(&speed);
    }

    gpsSensor.feedGPS();
    loopRate(2000);
}

void loopRate(int freq)
{
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
    while (invFreq > (checker - currentTime))
    {
        gpsSensor.feedGPS();
        checker = micros();
    }
}