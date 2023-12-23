#include "Eigen/Dense"
#include "Eigen/Sparse"
#define PI 3.14

using namespace Eigen;
using Eigen::VectorXf;
using Eigen::MatrixXf;


class QuadEstimatorEKF  {
  private:
    float rollEst, pitchEst, yawEst;
    
  public:
    MatrixXf Q;   // external noise matrix
    MatrixXf Q_at;
    MatrixXf R_GPS; // noise GPS measurement matrix
    MatrixXf R_Mag; // noise MAG measurment matrix
    MatrixXf ekfCov; // process noise matrix of the state
    MatrixXf ekfCov_at; // process noise matrix of the state
    VectorXf ekfState;  // state of ekf
    MatrixXf ekfCov_pred;  // predicted Cov matrix
    VectorXf xp;  // state for attitude estimation
    MatrixXf H_at;  // attitude estimation measurement matrix
    MatrixXf K_at;  // // attitude estimation kalman gain
    Vector3f acc;  // acc coming from accellerometer
    Vector3f gyro; // gyro measurment
    Vector3f estAttitude;  // attitude estimation vector with yaw, pitch and roll
    Vector4f ut;  // control vector
    MatrixXf R;  // noise measurment matrix
    MatrixXf R_at;
    VectorXf predState;  // predicted state
    Vector3f inertial_accel;
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

    VectorXf xt_at;  // attitude estimation quaternion state
   
    QuadEstimatorEKF(VectorXf ini_state, VectorXf ini_stdDevs, float AccX, float AccY, float AccZ, float GyroX, float GyroY, float GyroZ) {   // for ekf
      acc = Vector3f(AccX, AccY, AccZ);  // data coming from accelerometer
      gyro = Vector3f(GyroX, GyroY, GyroZ); // data coming from gyro
      ekfCov.setIdentity(Nstate, Nstate);

      ut(0) = acc.x();  //  control vector
      ut(1) = acc.y();
      ut(2) = acc.z();
      ut(3) = gyro.z();

      for (int i = 0; i < Nstate; i++)
        ekfCov(i, i) = ini_stdDevs(i) * ini_stdDevs(i);

      // load the transition model covariance
      Q.setZero(Nstate, Nstate);
      Q(0, 0) = Q(1, 1) = powf(QPosXYStd, 2);
      Q(2, 2) = powf(QPosZStd, 2);
      Q(3, 3) = Q(4, 4) = powf(QVelXYStd, 2);
      Q(5, 5) = powf(QVelZStd, 2);
      Q(6, 6) = powf(QYawStd, 2);
      Q *= dtIMU;

      predState.setZero(Nstate);

      R_GPS.setZero(6, 6);
      R_GPS(0, 0) = R_GPS(1, 1) = powf(GPSPosXYStd, 2);
      R_GPS(2, 2) = powf(GPSPosZStd, 2);
      R_GPS(3, 3) = R_GPS(4, 4) = powf(GPSVelXYStd, 2);
      R_GPS(5, 5) = powf(GPSVelZStd, 2);

      // magnetometer measurement model covariance
      R_Mag.setZero(Nstate, Nstate);
      R_Mag(0, 0) = powf(MagYawStd, 2);

      //attitude estimation
      xt_at.setZero(4);
      xt_at(0) = 1;
      ekfCov_at.setIdentity(4,4);

      rollEst = 0;
      pitchEst = 0;
      yawEst = 0;

      Q_at = Q_at.setIdentity(4,4) * 0.0001f;  // initialization of random noise matrix

      H_at.setIdentity(4,4);

      R_at = R_at.setIdentity(4,4)*0.0001f;

      // initial conditions of ekfState 
      ekfState = ini_state;
    }

    float Roll() {return this->rollEst;}

    float Pitch() {return this->pitchEst;}

    float Yaw() {return this->yawEst;}


    VectorXf Euler1232EP(Vector3f p) {  // from euler angle to quaternion in XYZ    
      VectorXf q(4);
      float c1 = cosf(p(0) / 2);
      float s1 = sinf(p(0) / 2);
      float c2 = cosf(p(1) / 2);
      float s2 = sinf(p(1) / 2);
      float c3 = cosf(p(2) / 2);
      float s3 = sinf(p(2) / 2);

      q(0) = c1*c2*c3-s1*s2*s3;
      q(1) = s1*c2*c3+c1*s2*s3;
      q(2) = c1*s2*c3-s1*c2*s3;
      q(3) = c1*c2*s3+s1*s2*c3;

      return q;
    }

    VectorXf Euler3212EP(Vector3f p) {  // from euler angle to quaternion in ZYX    
      VectorXf q(4);
      float c1 = cosf(p(0) / 2);
      float s1 = sinf(p(0) / 2);
      float c2 = cosf(p(1) / 2);
      float s2 = sinf(p(1) / 2);
      float c3 = cosf(p(2) / 2);
      float s3 = sinf(p(2) / 2);

      q(0) = c1*c2*c3+s1*s2*s3;
      q(1) = c1*c2*s3-s1*s2*c3;
      q(2) = c1*s2*c3+s1*c2*s3;
      q(3) = s1*c2*c3-c1*s2*s3;

      return q;
    }


    Vector3f EPEuler123(VectorXf q) {  // from quaternions to euler angles in XYZ
      float q0 = q(0);
      float q1 = q(1);
      float q2 = q(2);
      float q3 = q(3);

      float yaw = atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
      float pitch = asinf(2*(q1*q3 + q0*q2));
      float roll = atan2f(-2*(q1*q2-q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);

      return Vector3f(yaw, pitch, roll);
    }

    Vector3f EPEuler321(VectorXf q) {  // quaternions to euler in ZYX
      float q0 = q(0);
      float q1 = q(1);
      float q2 = q(2);
      float q3 = q(3);

      float yaw = atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
      float pitch = asinf(-2*(q1*q3-q0*q2));
      float roll = atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);

      return Vector3f(yaw, pitch, roll);
    }

    void attitude_integration(float dt) {
         // Gyroscope rates
      double wx = gyro.x();
      double wy = gyro.y();
      double wz = gyro.z();

    // Quaternion rates
      VectorXf qDot(4);
      qDot << 0.5 * (-wx * xt_at(1) - wy * xt_at(2) - wz * xt_at(3)),
            0.5 * (wx * xt_at(0) + wy * xt_at(3) - wz * xt_at(2)),
            0.5 * (-wx * xt_at(3) + wy * xt_at(0) + wz * xt_at(1)),
            0.5 * (wx * xt_at(2) - wy * xt_at(1) + wz * xt_at(0));

      xt_at = xt_at + qDot * dt;
    }

    void kf_attitudeEstimation(float dt) {
      MatrixXf A(4, 4);
      MatrixXf B(4, 4);
      VectorXf z(4);      
      
      // estimating roll and pitch with accellerometer
      float accelPitch = asinf(-acc.x() / (-9.81f));
      float accelRoll = asinf(acc.y() / (-9.81f * cos(estAttitude(1)) ) );

      B << 0, -gyro.x(), -gyro.y(), -gyro.z(),
      gyro.x(), 0, gyro.z(), -gyro.y(),
      gyro.y(), -gyro.z(), 0, gyro.x(),
      gyro.z(), gyro.y(), -gyro.x(), 0;

      // need to do a kf for estimating roll, pitch and yaw. Roll->phi, Pitch->teta, Yaw->psi
      A = A.setIdentity() + dt * .5f * B;
      xp = A * xt_at;
      xt_at = xp;
      //xt_at(1) = -xt_at(1);
      //xt_at(3) = -xt_at(3);
      /*ekfCov_pred = A * ekfCov_at * A.transpose() + Q_at;
      K_at = ekfCov_pred * H_at.transpose() * (H_at * ekfCov_pred * H_at.transpose() + R_at).inverse();
     
      Vector3f p = Vector3f(estAttitude(0), accelPitch, accelRoll);
      z = Euler1232EP(p);
      xt_at = xp + K_at * (z - H_at * xp);
      ekfCov_at = ekfCov_pred - K_at * H_at * ekfCov_pred;

      xt_at = xp;                       //using only gyro integration for testing                                                       
      estAttitude = EPEuler123(xt_at);  // attitude vector with euler angles   
      ekfState(6) = estAttitude(0);*/
    
      // complimentary filter part
      //float accelRoll = atan2f(acc.y(), acc.z());
      //float accelPitch = atan2f(-acc.x(), -9.81f);
      Vector3f predictedAttitude = EPEuler321(xp); 
      
      // FUSE INTEGRATION AND UPDATE
      /*estAttitude(2) = attitudeTau / (attitudeTau + dt) * (predictedAttitude(2))+dt / (attitudeTau + dt) * accelRoll;
      //estAttitude(2) = predictedAttitude(2);
      estAttitude(1) = attitudeTau / (attitudeTau + dt) * (predictedAttitude(1))+dt / (attitudeTau + dt) * accelPitch;
      estAttitude(0) = predictedAttitude(0);
      xt_at = Euler3212EP(estAttitude); */
      
    }

    void complimentary_filter_attitude_estimation(float dt){
      Vector3f dq = BodyRates_to_EulerVelocities(Vector3f(gyro(2), gyro(1), gyro(0)));

      float predictedRoll = rollEst + dt * dq(0);
      float predictedPitch = pitchEst + dt * dq(1);
      ekfState(6) = ekfState(6) + dt * dq(2);
      

      /////////////////////////////// END STUDENT CODE ////////////////////////////
      
      // CALCULATE UPDATE
      double accelRoll = atan2f(acc.y(), acc.z());
      double accelPitch = atan2f(-acc.x(), +9.81f);

      // FUSE INTEGRATION AND UPDATE
      rollEst = attitudeTau / (attitudeTau + dt) * (predictedRoll)+dt / (attitudeTau + dt) * accelRoll;
      pitchEst = attitudeTau / (attitudeTau + dt) * (predictedPitch)+dt / (attitudeTau + dt) * accelPitch;
      
      estAttitude(0) = ekfState(6);
      estAttitude(1) = pitchEst;
      estAttitude(2) = rollEst; 
      xt_at = Euler1232EP(estAttitude);
    }

    Vector3f EulerVelocities_to_BodyRates(Vector3f omega){
      Matrix3f m;
      m(0, 0) = 1;
      m(1, 0) = 0;
      m(2, 0) = 0;
      m(0, 1) = sin(rollEst) * tan(pitchEst);
      m(0, 2) = cos(rollEst) * tan(pitchEst);
      m(1, 1) = cos(rollEst);
      m(1, 2) = -sin(rollEst);
      m(2, 1) = sin(rollEst) / cos(pitchEst);
      m(2, 2) = cos(rollEst) / cos(pitchEst);
      return m.inverse()*omega;
    }

    Vector3f BodyRates_to_EulerVelocities(Vector3f pqr){
      Matrix3f m;
      m(0, 0) = 1;
      m(1, 0) = 0;
      m(2, 0) = 0;
      m(0, 1) = sin(rollEst) * tan(pitchEst);
      m(0, 2) = cos(rollEst) * tan(pitchEst);
      m(1, 1) = cos(rollEst);
      m(1, 2) = -sin(rollEst);
      m(2, 1) = sin(rollEst) / cos(pitchEst);
      m(2, 2) = cos(rollEst) / cos(pitchEst);
      return m*pqr;
    }

    MatrixXf Rot_mat() {  // rotational matrix from body frame to world frame
      MatrixXf R(3, 3);
  
      R(0, 0) = cos(estAttitude(1)) * cos(estAttitude(0));
      R(0, 1) = sin(estAttitude(2)) * sin(estAttitude(1)) * cos(estAttitude(0)) - cos(estAttitude(2)) * sin(estAttitude(0));
      R(0, 2) = cos(estAttitude(2)) * sin(estAttitude(1)) * cos(estAttitude(0)) + sin(estAttitude(2)) * sin(estAttitude(0));
      R(1, 0) = cos(estAttitude(1)) * sin(estAttitude(0));
      R(1, 1) = sin(estAttitude(2)) * sin(estAttitude(1)) * sin(estAttitude(0)) + cos(estAttitude(2)) * cos(estAttitude(0));
      R(1, 2) = cos(estAttitude(2)) * sin(estAttitude(1)) * sin(estAttitude(0)) - sin(estAttitude(2)) * cos(estAttitude(0));
      R(2, 0) = -sin(estAttitude(1));
      R(2, 1) = cos(estAttitude(1)) * sin(estAttitude(2));
      R(2, 2) = cos(estAttitude(1)) * cos(estAttitude(2));

      return R;
    }

    MatrixXf GetRbgPrime() {
       MatrixXf RbgPrime(3, 3);
       RbgPrime.setZero();
      
       RbgPrime(0, 0) = -cos(estAttitude(1)) * sin(estAttitude(0));
       RbgPrime(0, 1) = -sin(estAttitude(2)) * sin(estAttitude(1)) * sin(estAttitude(0)) - cos(estAttitude(2)) * cos(estAttitude(0));
       RbgPrime(0, 2) = -cos(estAttitude(2)) * sin(estAttitude(1)) * sin(estAttitude(0)) + sin(estAttitude(2)) * cos(estAttitude(0));
       RbgPrime(1, 0) = cos(estAttitude(1)) * cos(estAttitude(0));
       RbgPrime(1, 1) = sin(estAttitude(2)) * sin(estAttitude(1)) * cos(estAttitude(0)) - cos(estAttitude(2)) * sin(estAttitude(0));
       RbgPrime(1, 2) = cos(estAttitude(2)) * sin(estAttitude(1)) * cos(estAttitude(0)) + sin(estAttitude(2)) * sin(estAttitude(0));

       return RbgPrime;
    }

    Matrix3f quatRotMat(VectorXf q) {
      Matrix3f M;

      M << 1 - 2*q(2)*q(2) - 2*q(3)*q(3), 2*q(1)*q(2) - 2*q(0)*q(3), 2*q(1)*q(3) + 2*q(0)*q(2),
           2*q(1)*q(2) + 2*q(0)*q(3),  1 - 2*q(1)*q(1) - 2*q(3)*q(3), 2*q(2)*q(3) - 2*q(0)*q(1),
           2*q(1)*q(3) - 2*q(0)*q(2), 2*q(2)*q(3) + 2*q(0)*q(1), 1 - 2*q(1)*q(1) - 2*q(2)*q(2);
      return M;
    }

    Matrix3f quatRotMat_2(VectorXf q){
      Matrix3f m;
      q = q/q.norm();
      Vector3f q_v = q.head(3);

      Matrix3f q_cross;
      q_cross <<   0, -q(2),  q(1),
                   q(2),  0, -q(0),
                   -q(1), q(0),  0;
      Matrix3f eye;
      eye.setIdentity();
      m = (q(3) - (q_v.transpose())*q_v)*eye + 2*q_v*(q_v.transpose()) - 2*q(3)*q_cross;

      return m;
    }

    void predict(float dt) {
      MatrixXf R_bg(3, 3);
      MatrixXf R_bg_quat(3, 3);
      MatrixXf A(Nstate, Nstate);   
      MatrixXf RbgPrime(3,3);

      A.setIdentity();
      A(0, 3) = A(1, 4) = A(2, 5) = dt;

      MatrixXf m(Nstate, 3);
      m.setZero();

      R_bg_quat = quatRotMat_2(xt_at).transpose();
      
      for(int i = 0; i < 3; i++) {
          m(3, i) = R_bg(0, i);
          m(4, i) = R_bg(1, i);
          m(5, i) = R_bg(2, i);
       }

      MatrixXf B(Nstate, 3); 
      B = m * dt;

      VectorXf ut_2(3);
      ut_2(0) = ut(0); 
      ut_2(1) = ut(1); 
      ut_2(2) = ut(2); 
      inertial_accel = R_bg_quat * ut_2;


      predState = A * predState + B * ut_2;  // predicting state
      ekfState = predState;
      
      RbgPrime = GetRbgPrime();

      MatrixXf gPrime(Nstate, Nstate);
      gPrime.setIdentity();
      MatrixXf m1(Nstate, Nstate);

      float sv1 = 0.0f, sv2 = 0.0f, sv3 = 0.0f;

      for (int i = 0; i < 3; i++) {
        sv1 += RbgPrime(0, i) * ut_2(i);   // scalar product of RbgPrime with control vector
        sv2 += RbgPrime(1, i) * ut_2(i);
        sv3 += RbgPrime(2, i) * ut_2(i);
      }

      gPrime(3, Nstate - 1) = sv1 * dt;
      gPrime(4, Nstate - 1) = sv2 * dt;
      gPrime(5, Nstate - 1) = sv3 * dt;
      gPrime(0, 3) = gPrime(1, 4) = gPrime(2, 5) = dt;
      
      m1 = gPrime * ekfCov;

      MatrixXf m2(Nstate, Nstate);
      gPrime.transposeInPlace();
      m2 = m1 * gPrime;

      ekfCov_pred = m2 + Q;
    }

    VectorXf PredictState(VectorXf curState, float dt, Vector3f accel, Vector3f gyro){
  
    VectorXf predictedState = curState;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    MatrixXf R_bg(3, 3);
    R_bg = quatRotMat(xt_at);
    inertial_accel = R_bg*accel;
    inertial_accel(2) = inertial_accel(2) + 9.81; //remove gravity

    predictedState(0) = curState(0) + curState(3)* dt;
    predictedState(1) = curState(1) + curState(4) * dt;
    predictedState(2) = curState(2) + curState(5) * dt;
    predictedState(3) = curState(3) + inertial_accel(0) * dt;
    predictedState(4) = curState(4) + inertial_accel(1) * dt;
    predictedState(5) = curState(5) + inertial_accel(2) * dt;
    
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return predictedState;
    }
    void predict_2(float dt)
    {
    // predict the state forward
    VectorXf newState = PredictState(ekfState, dt, acc, gyro);

    // Predict the current covariance forward by dt using the current accelerations and body rates as input.
    // INPUTS: 
    //   dt: time step to predict forward by [s]
    //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
    //   gyro: body rates of the vehicle, in body frame [rad/s]
    //   state (member variable): current state (state at the beginning of this prediction)
    //   
    // OUTPUT:
    //   update the member variable cov to the predicted covariance

    // HINTS
    // - update the covariance matrix cov according to the EKF equation.
    // 
    // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
    //
    // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
    //
    // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
    // 
    // - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
    //   1) Calculate the necessary helper matrices, building up the transition jacobian
    //   2) Once all the matrices are there, write the equation to update cov.
    //
    // - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
    //   

    // we'll want the partial derivative of the Rbg matrix
    MatrixXf RbgPrime = GetRbgPrime();

    // we've created an empty Jacobian for you, currently simply set to identity
    MatrixXf gPrime(7,7);
    gPrime.setIdentity();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    gPrime(0, 3) = dt;
    gPrime(1, 4) = dt;
    gPrime(2, 5) = dt;

    VectorXf helper_matrix = RbgPrime * acc;
    gPrime(3, 6) = helper_matrix(0) * dt;
    gPrime(4, 6) = helper_matrix(1) * dt;
    gPrime(5, 6) = helper_matrix(2) * dt;
    MatrixXf gTranspose = gPrime.transpose().eval();
    ekfCov = gPrime * ekfCov * gTranspose + Q;
    
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    ekfState = newState;
    }

    void update_ekf(VectorXf z, MatrixXf H, MatrixXf R, VectorXf zFromX, float dt) {
        assert(z.size() == H.rows());
        assert(Nstate == H.cols());
        assert(z.size() == R.rows());
        assert(z.size() == R.cols());
        assert(z.size() == zFromX.size());

        MatrixXf toInvert(z.size(), z.size());
        toInvert = H*ekfCov*H.transpose() + R;
        MatrixXf K = ekfCov * H.transpose() * toInvert.inverse();

        ekfState = ekfState + K*(z - zFromX);

        MatrixXf eye(Nstate, Nstate);
        eye.setIdentity();

        ekfCov = (eye - K*H)*ekfCov;
    }

    void updateFromMag(float dt, float magYaw) {  
      VectorXf z(1), zFromX(1);
      z(0) = magYaw;  // measure done by the mag, magYaw taken by the magnatometer
      zFromX(0) = ekfState(6);

      MatrixXf hPrime(1, Nstate);
      hPrime.setZero();
      hPrime(0, 6) = 1;
      float QYawStd = 2.0f;
  
      if (abs(z(0) - zFromX(0)) > PI) 
        if (z(0) < zFromX(0))
            z(0) += 2.f * PI;
        else
            z(0) -= 2.f * PI;
        update_ekf(z, hPrime, R_Mag, zFromX, dt);
      }

    void updateFromGps(Vector3f pos, Vector3f vel, float dt) {

      VectorXf z(6), zFromX(6);
      z(0) = pos.x();
      z(1) = pos.y();
      z(2) = pos.z();
      z(3) = vel.x();
      z(4) = vel.y();
      z(5) = vel.z();

      MatrixXf hPrime(6, Nstate);
      hPrime.setZero();

      for (int i = 0; i < 6; i++) {
          for (int j = 0; j < Nstate; j++) {
            if (i == j)
              hPrime(i, j) = 1;
          }
      }

      for (int i = 0; i < 6; i++) 
        zFromX(i) = ekfState(i);

      update_ekf(z, hPrime, R_GPS, zFromX, dt);
    }  
};
