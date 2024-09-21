#include <Arduino.h>

#include "QuadEstimatorEKF.h"
#include "baro.h"
#include "gps.h"
#include "imu.h"
#include "mag.h"
#include "motor.h"
#include "pinDef.h"
#include "radio.h"
#include "utils.h"

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
quat_t quat;
attitude_t att;
bar_t bar;
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
QuadEstimatorEKF estimation;

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
    initializeImu();
    validGPS = initializeGPS(115200, &coordGPS);
    validMag = initializeMag();
    validBaro = initializeBarometer();
    // initializeMotors();
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
    Serial.println("Initialization done");
}

void loop()
{
    prevTime = currentTime;
    currentTime = micros();

    // Getting values from imu
    if (micros() - accIMUFrame.t >= 5000)
    { // 200Hz
        getAcceleration(&accIMUFrame);
        if (DEBUG_ACC || DEBUG_ALL)
        {
            Serial.print("Acc:\t");
            printData(&accIMUFrame); 
        }
    }

    if (micros() - gyro.t >= 5000)
    { // 200Hz
        getGyro(&gyro);
        if (DEBUG_GYRO || DEBUG_ALL)
        {
            Serial.print("Gyro:\t");
            printData(&gyro);
        }
    }

    if (validGPS && getGPS(&coordGPS, &speedGPS))
    {
        /*posGPS.x = r * coordGPS.lat - posGPS0.x;             // north
        posGPS.y = r * coordGPS.lon * cos(lat0) - posGPS0.y; // east
        posGPS.z = -coordGPS.alt + posGPS0.z;                // down
        posGPS.dt = coordGPS.dt; */

        posGPS.x = 111320 * (coordGPS.lat - lat0);                         // north
        posGPS.y = 111320 * cos(lat0) * (coordGPS.lon - lon0);             // east
        posGPS.z = coordGPS.alt - alt0;                                    // down*/
        posGPS.dt = coordGPS.dt;

       estimation.updateFromGps(Vector3f(posGPS.x, posGPS.y, posGPS.z), Vector3f(speedGPS.x, speedGPS.y, speedGPS.z), posGPS.dt / 1000.0f);

        if (DEBUG_GPS || DEBUG_ALL)
        {
            Serial.print("GPS_Pos:\t");
            printData(&posGPS);
            /*Serial.print("GPS_LAT:\t");
            Serial.print(coordGPS.lat, 6);
            Serial.print("\tGPS_LON:\t");
            Serial.println(coordGPS.lon, 6);*/
            Serial.print("GPS_Speed:\t");
            printData(&speedGPS);
        }
    }

    if (validMag && micros() - mag.t > 5000)
    { // 200Hz
        getMag(&mag);

        yawMag = estimation.yawFromMag(mag, quat);
        estimation.updateFromMag(yawMag, mag.dt / 1000.0f);

        if (DEBUG_MAG || DEBUG_ALL)
        {
            Serial.print("Mag:\t");
            printData(&mag);
        }
    }

    if (validBaro && micros() - bar.t > 5000)
    {  // 200Hz
        getBarometer(&bar);

        estimation.updateFromBar(bar.altitude, bar.dt / 1000.0f);

        if (DEBUG_BAR || DEBUG_ALL)
        {
            Serial.print("Bar:\t");
            printData(&bar);
        }
    }

    // // removing the angular offset

    accBodyFrame = R * Vector3f(accIMUFrame.x, accIMUFrame.y, accIMUFrame.z); // acceleration in drone frame
    // // EKF estimation for attitude, speed and position
    // // estimation.kf_attitudeEstimation(accBodyFrame, Vector3f(gyro.x, gyro.y, gyro.z), accIMUFrame.dt);  // quaternion attitude estimation
    getQuaternion(&quat);
    getAttitude(&att);
    estimation.xt_at << quat.w, quat.x, quat.y, quat.z; // just copy quaternion from dmp
    estimation.estAttitude = estimation.EPEuler321(estimation.xt_at);
    estimation.predict(accBodyFrame, Vector3f(gyro.x, gyro.y, gyro.z), accIMUFrame.dt / 1000.0f); // prediction of the (x, y, z) position and velocity
    estimation.getPosVel(&pos, &speed);

    if (DEBUG_QUAT || DEBUG_ALL)
    {
        Serial.print("EKF_Quat:\t");
        printData(&quat);
    }
    if (DEBUG_MAGYAW || DEBUG_ALL)
    {
        // Serial.print("YAW_mag:\t");
        // Serial.print(yawMag);   //da problemi con serialtomat.py
    }
    if (DEBUG_YPR || DEBUG_ALL)
    {
        Serial.print("EKF_YPR:\t");
        printData(&att);
    }
    if (DEBUG_POS || DEBUG_ALL)
    {
        Serial.print("EKF_Pos:\t");
        printData(&pos);
    }
    if (DEBUG_VEL || DEBUG_ALL)
    {
        Serial.print("EKF_Speed:\t");
        printData(&speed);
    }

    feedGPS();
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
        feedGPS();
        checker = micros();
    }
}