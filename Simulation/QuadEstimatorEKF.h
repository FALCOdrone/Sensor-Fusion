#ifndef EKF_H
#define EKF_H

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "types.h"
#define PI 3.14

using namespace Eigen;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class QuadEstimatorEKF  {
  private:
    float rollEst, pitchEst, yawEst;
    MatrixXf Q;   // external noise matrix
    MatrixXf Q_at;
    MatrixXf R_GPS; // noise GPS measurement matrix
    MatrixXf R_Mag; // noise MAG measurment matrix
    MatrixXf H_at;  // attitude estimation measurement matrix
    MatrixXf R;  // noise measurment matrix
    MatrixXf R_at;
    
    float dtIMU = 0.01f;
    float attitudeTau = 0.5;

    const int Nstate = 7;
    
    float QPosXYStd = .5f;
    float QPosZStd = .5f;
    float QVelXYStd = .5f;
    float QVelZStd = .5f;
    float QYawStd = .095f;

    float GPSPosXYStd = .1f;
    float GPSPosZStd = .3f;
    float GPSVelXYStd = .1f;
    float GPSVelZStd = .3f;

    float MagYawStd = .1f;

    VectorXf Euler1232EP(Vector3f p);
    VectorXf Euler3212EP(Vector3f p);
    Vector3f EPEuler123(VectorXf q);
    Vector3f EPEuler321(VectorXf q);
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
   
    QuadEstimatorEKF(VectorXf ini_state, VectorXf ini_stdDevs);
    
    void kf_attitudeEstimation(Vector3f acc, Vector3f gyro, float dt);
    void complimentary_filter_attitude_estimation(float dt);
    VectorXf predict(Vector3f acc, Vector3f gyro, float dt);

    void updateFromMag(float magYaw, float dt);
    void updateFromGps(Vector3f pos, Vector3f vel, float dt);
    
    //added to integrate code with FALCO.ino
    void getAttitude(quat_t *quat, attitude_t *attitude);
    void getPosVel(vec_t *pos, vec_t *vel);
    
    // added for computing mag readings 
    VectorXf getMagReadings(vec_t *magB, quat_t *quat);
};
#endif