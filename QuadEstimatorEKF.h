#ifndef EKF_H
#define EKF_H

//#include "Eigen/Dense"
//#include "Eigen/Sparse"
#include <ArduinoEigen.h>  // Eigen by hideakitai
#include "types.h"
#include <Arduino.h>
//#define PI 3.14

//using namespace Eigen;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix3f;
using Eigen::Vector3f;

class QuadEstimatorEKF  {
  private:
    float rollEst, pitchEst, yawEst;
    MatrixXf Q;   // external noise matrix
    MatrixXf Q_at;
    MatrixXf R_GPS; // noise GPS measurement matrix
    MatrixXf R_Mag; // noise MAG measurment matrix
    MatrixXf R_bar; // noise BAR measurment matrix
    MatrixXf H_at;  // attitude estimation measurement matrix
    MatrixXf R;  // noise measurment matrix
    MatrixXf R_at;
    
    float dtIMU = 0.01f;
    float attitudeTau = 0.5;

    const int Nstate = 7;
    
    float QPosXYStd = .5f;
    float QPosZStd = .05f;
    float QVelXYStd = .5f;
    float QVelZStd = .05f;
    float QYawStd = .095f;

    //float GPSPosXYStd = .1f;
    float GPSPosXStd = .16f;
    float GPSPosYStd = .3f;
    float GPSPosZStd = 1.1f;
    float GPSVelXYStd = pow(0.1, 4);
    float GPSVelZStd =  pow(0.1, 3);
    //float GPSVelXYStd = .1f;
    //float GPSVelZStd = .3f;

    float MagYawStd = 1.2350f;

    VectorXf Euler1232EP(Vector3f p);
    VectorXf Euler3212EP(Vector3f p);
    Vector3f EPEuler123(VectorXf q);
    Vector3f EulerVelocities_to_BodyRates(Vector3f omega);
    Vector3f BodyRates_to_EulerVelocities(Vector3f pqr);
    MatrixXf Rot_mat();
    MatrixXf GetRbgPrime();
    Matrix3f quatRotMat(VectorXf q);
    Matrix3f quatRotMat_2(VectorXf q);

    void update_ekf(VectorXf z, MatrixXf H, MatrixXf R, VectorXf zFromX, float dt);   //general update formula, it is called by updatefromgps and updatefrommag


  public:
    MatrixXf ekfCov; // process noise matrix of the state
    MatrixXf ekfCov_at; // process noise matrix of the state
    VectorXf ekfState;  // state of ekf
    MatrixXf ekfCov_pred;  // predicted Cov matrix
    MatrixXf K_at;  // // attitude estimation kalman gain
    
    Vector3f estAttitude;  // attitude estimation vector with yaw, pitch and roll    
    VectorXf xt_at;  // attitude estimation quaternion state

    //QuadEstimatorEKF();
    void initialize(VectorXf ini_state, VectorXf ini_stdDevs);
    
    void kf_attitudeEstimation(Vector3f acc, Vector3f gyro, float dt);
    void complimentary_filter_attitude_estimation(Vector3f acc, Vector3f gyro, float dt);
    void predict(Vector3f acc, Vector3f gyro, float dt);

    void updateFromMag(float magYaw, float dt);
    void updateFromGps(Vector3f pos, Vector3f vel, float dt);
    void updateFromBar(float P, float dt);
    
    //added to integrate code with FALCO.ino
    void getAttitude(quat_t *quat, attitude_t *attitude);
    void getPosVel(vec_t *pos, vec_t *vel);
    
    // computing yaw from mag readings 
    float yawFromMag(vec_t mag, quat_t quat);

    // computing altitude from bar readings
    float zFromBar(float pressure);

    Vector3f EPEuler321(VectorXf q);    
};
#endif