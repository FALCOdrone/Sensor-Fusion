#include <Arduino.h>

#include "imu.h"
#include "QuadEstimatorEKF.h"
#include "motor.h"
#include "pinDef.h"
#include "radio.h"
#include "utils.h"
#include "gps.h"
#include "mag.h"

// variables
vec_t accelWithOffset;
VectorXf accelWithOffset2(3);
vec_t gyro;
VectorXf fixed_accel(3);
vec_t posGPS;
vec_t posGPS0;    //position (from gps) of starting point
gps_t coordGPS;
vec_t speedGPS;
vec_t mag;
vec_t pos;
vec_t speed;
quat_t quat;
attitude_t att;
float yawMag;
float lat0; //latitude at starting point, used for projection (lat long -> x y)
float r = 6371000; //earth radius (m)

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
    ini_state.setZero();
    ini_stdDevs.setOnes();
    estimation.initialize(ini_state, ini_stdDevs);
    initializeImu();
    initializeGPS();
    initializeMag();
    //initializeMotors();
    //initializeRadio();

    // setting initial values for estimation parameters/variables
    R << cos(PI/4), sin(PI/4), 0,
         -sin(PI/4), cos(PI/4), 0,
         0, 0, 1;
    getGPS(&coordGPS, &speedGPS);
    lat0 = coordGPS.lat;  
    posGPS0.x = r*coordGPS.lat; //north
    posGPS0.y = r*coordGPS.lon*cos(lat0); //east
    posGPS0.z = coordGPS.alt; //up
    Serial.println("Initialization done");
}

void loop() {

    prevTime = currentTime;
    currentTime = micros();
    
    // getting values from imu
    getAcceleration(&accelWithOffset);
    getGyro(&gyro);
    getGPS(&coordGPS, &speedGPS); 
    getMag(&mag); 
    
    // removing the angular offset
    accelWithOffset2(0) = accelWithOffset.x;
    accelWithOffset2(1) = accelWithOffset.y;
    accelWithOffset2(2) = accelWithOffset.z;
    
    fixed_accel = R*accelWithOffset2;  // body frame accelleration without offset
    yawMag = estimation.yawFromMag(mag, quat);
    
    posGPS.x = r*coordGPS.lat - posGPS0.x;  //north
    posGPS.y = r*coordGPS.lon*cos(lat0) - posGPS0.y;  //east
    posGPS.z = -coordGPS.alt + posGPS0.z;  //down
    posGPS.dt = coordGPS.dt;
    // EKF estimation for attitude, speed and position
    estimation.kf_attitudeEstimation(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // quaternion attitude estimation
    estimation.getAttitude(&quat, &att);
    estimation.predict(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // prediction of the (x, y, z) position and velocity
    
    //compute the update from gps
    if (isGPSUpdated()) {
        estimation.updateFromGps(Vector3f(posGPS.x, posGPS.y, posGPS.z), Vector3f(speedGPS.x, speedGPS.y, speedGPS.z), posGPS.dt);
    }
    estimation.updateFromMag(yawMag, mag.dt);  // TODO: calculate yaw from magnetometer data

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
    printIMUData(&att);
    printIMUData(&pos, "m");
    printIMUData(&speed, "m/s");
    
}
