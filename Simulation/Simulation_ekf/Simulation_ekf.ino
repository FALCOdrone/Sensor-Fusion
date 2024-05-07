#include <Arduino.h>

#include "imu.h"
#include "QuadEstimatorEKF.h"
#include "motor.h"
#include "pinDef.h"
#include "radio.h"
#include "utils.h"

// variables
vec_t accelWithOffset;
VectorXf accelWithOffset2(3);
vec_t gyro;
VectorXf fixed_accel(3);
vec_t pos;
vec_t speed;
quat_t quat;
attitude_t att;

// Parameters for EKF
const int Nstate = 7;
VectorXf ini_state(Nstate);
MatrixXf ini_stdDevs;
VectorXf predict_state(Nstate);

// timing parameters
unsigned long currentTime, prevTime;
int GPSrate = 1;

// initialization of the constructor for estimation
QuadEstimatorEKF estimation(ini_state, ini_stdDevs);

void setup() {
    Serial.begin(115200);
    initializeImu();
    initializeMotors();
    initializeRadio();

    // setting initial values for estimation parameters/variables
    ini_state.setZero();
    ini_stdDevs.setIdentity(Nstate, Nstate);
}

void loop() {

    prevTime = currentTime;
    currentTime = micros();
    
    // getting values from imu
    getAcceleration(&accelWithOffset);
    getGyro(&gyro);

    // removing the angular offset
    accelWithOffset2(0) = accelWithOffset.x;
    accelWithOffset2(1) = accelWithOffset.y;
    accelWithOffset2(2) = accelWithOffset.z;
    MatrixXf R(3, 3);
    R << cos(PI/4), sin(PI/4), 0,
      -sin(PI/4), cos(PI/4), 0,
      0, 0, 1;
    fixed_accel = R*accelWithOffset2;  // body frame accelleration without offset
    
    // EKF estimation for attitude, speed and position
    estimation.kf_attitudeEstimation(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // quaternion attitude estimation
    estimation.getAttitude(&quat, &att);
    predict_state = estimation.predict(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // prediction of the (x, y, z) position and velocity
    // probably is better declaring the predict as a void function

    // compute the update from gps
    
    // TODO: get data from GPS and Mag
    if (currentTime - prevTime > (1.0 / GPSrate * 1000000.0)) {
        // getGPS(&pos, &speed);  // Updates GPS data (m)
        // getMag(&mag);          // Updates magnetometer data (uT)
    }

    estimation.getPosVel(&pos, &speed);
}
