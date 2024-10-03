#include "QuadEstimatorEKF.h"


void EKF::initialize(VectorXf ini_state, VectorXf ini_stdDevs) {   // for ekf

  ekfCov.setIdentity(Nstate, Nstate);

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

  R_GPS.setZero(6, 6);
  R_GPS(0, 0) = powf(GPSPosXStd, 2);
  R_GPS(1, 1) = powf(GPSPosYStd, 2);
  R_GPS(2, 2) = powf(GPSPosZStd, 2);
  R_GPS(3, 3) = R_GPS(4, 4) = powf(GPSVelXYStd, 2);
  R_GPS(5, 5) = powf(GPSVelZStd, 2);

  // magnetometer measurement model covariance
  R_Mag.setZero(1,1);
  R_Mag(0, 0) = powf(MagYawStd, 2);
  R_bar.setZero(1,1);
  R_bar(0, 0) = powf(0.2f, 2);

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

VectorXf EKF::Euler1232EP(Vector3f p) {  // from euler angle to quaternion in XYZ    
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


VectorXf EKF::Euler3212EP(Vector3f p) {  // from euler angle to quaternion in ZYX    
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

Vector3f EKF::EPEuler123(VectorXf q) {  // from quaternions to euler angles in XYZ
  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  float yaw = atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
  float pitch = asinf(2*(q1*q3 + q0*q2));
  float roll = atan2f(-2*(q1*q2-q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);

  return Vector3f(yaw, pitch, roll);
}

Vector3f EKF::EPEuler321(VectorXf q) {  // quaternions to euler in ZYX
  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  float yaw = atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
  float pitch = asinf(-2*(q1*q3-q0*q2));
  float roll = atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);

  return Vector3f(yaw, pitch, roll);
}

void EKF::kf_attitudeEstimation(Vector3f acc, Vector3f gyro, float dt) {
  MatrixXf A(4, 4);
  MatrixXf B(4, 4);
  VectorXf z(4);      
  VectorXf xp(4);      
  
  // estimating roll and pitch with accellerometer
  float accelPitch = asinf(-acc.x() / (-9.81f));
  float accelRoll = asinf(acc.y() / (-9.81f * cos(accelPitch) ) );

  B << 0, -gyro.x(), -gyro.y(), -gyro.z(),
  gyro.x(), 0, gyro.z(), -gyro.y(),
  gyro.y(), -gyro.z(), 0, gyro.x(),
  gyro.z(), gyro.y(), -gyro.x(), 0;

  /*  other convention 
    B << 0, -gyro.x(), -gyro.y(), -gyro.z(),
        gyro.x(), 0, -gyro.z(), gyro.y(),
        gyro.y(), gyro.z(), 0, -gyro.x(),
        gyro.z(), -gyro.y(), gyro.x(), 0;
  */
  
  // prediction step
  A = A.setIdentity() + dt * .5f * B;
  xp = A * xt_at;
  
  ekfCov_pred = A * ekfCov_at * A.transpose() + Q_at;
  K_at = ekfCov_pred * H_at.transpose() * (H_at * ekfCov_pred * H_at.transpose() + R_at).inverse();
  
  // update step
  Vector3f p = Vector3f(estAttitude(0), accelPitch, accelRoll);
  z = Euler3212EP(p);
  xt_at = xp + K_at * (z - H_at * xp);
  ekfCov_at = ekfCov_pred - K_at * H_at * ekfCov_pred;
                                                
  estAttitude = EPEuler321(xt_at);  // attitude vector with euler angles yaw pitch roll  
  ekfState(6) = estAttitude(0);      
}

void EKF::complimentary_filter_attitude_estimation(Vector3f acc, Vector3f gyro, float dt){
  Vector3f dq = BodyRates_to_EulerVelocities(Vector3f(gyro(2), gyro(1), gyro(0)));

  float predictedRoll = rollEst + dt * dq(0);
  float predictedPitch = pitchEst + dt * dq(1);
  ekfState(6) = ekfState(6) + dt * dq(2);
  
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

Vector3f EKF::BodyRates_to_EulerVelocities(Vector3f pqr){
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

MatrixXf EKF::GetRbgPrime() {
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

Matrix3f EKF::quatRotMat(VectorXf q) {
  Matrix3f M;

  M << 1 - 2*q(2)*q(2) - 2*q(3)*q(3), 2*q(1)*q(2) - 2*q(0)*q(3), 2*q(1)*q(3) + 2*q(0)*q(2),
        2*q(1)*q(2) + 2*q(0)*q(3),  1 - 2*q(1)*q(1) - 2*q(3)*q(3), 2*q(2)*q(3) - 2*q(0)*q(1),
        2*q(1)*q(3) - 2*q(0)*q(2), 2*q(2)*q(3) + 2*q(0)*q(1), 1 - 2*q(1)*q(1) - 2*q(2)*q(2);
  return M;
}

Matrix3f EKF::quatRotMat_2(VectorXf q){
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

void EKF::predict(Vector3f acc, Vector3f gyro, float dt){

    VectorXf predictedState = ekfState;
    Vector3f inertial_accel;
    MatrixXf R_bg(3, 3);
    R_bg = quatRotMat(xt_at);
    inertial_accel = R_bg*acc;
    inertial_accel(2) = inertial_accel(2); //remove gravity

    predictedState(0) = ekfState(0) + ekfState(3)* dt;
    predictedState(1) = ekfState(1) + ekfState(4) * dt;
    predictedState(2) = ekfState(2) + ekfState(5) * dt;
    predictedState(3) = ekfState(3) + inertial_accel(0) * dt;
    predictedState(4) = ekfState(4) + inertial_accel(1) * dt;
    predictedState(5) = ekfState(5) + inertial_accel(2) * dt;

    // we'll want the partial derivative of the Rbg matrix
    MatrixXf RbgPrime = GetRbgPrime();

    // we've created an empty Jacobian for you, currently simply set to identity
    MatrixXf gPrime(7,7);
    gPrime.setIdentity();

    gPrime(0, 3) = dt;
    gPrime(1, 4) = dt;
    gPrime(2, 5) = dt;

    VectorXf helper_matrix = RbgPrime * acc;
    gPrime(3, 6) = helper_matrix(0) * dt;
    gPrime(4, 6) = helper_matrix(1) * dt;
    gPrime(5, 6) = helper_matrix(2) * dt;
    MatrixXf gTranspose = gPrime.transpose().eval();
    
    ekfCov = gPrime * ekfCov * gTranspose + Q;
    ekfState = predictedState;
}

void EKF::update_ekf(VectorXf z, MatrixXf H, MatrixXf R, VectorXf zFromX, float dt) {
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

void EKF::updateFromMag(float magYaw, float dt) {  
  VectorXf z(1), zFromX(1);
  z(0) = magYaw;  // measure done by the mag, magYaw taken by the magnatometer
  zFromX(0) = ekfState(6);

  MatrixXf hPrime(1, Nstate);
  hPrime.setZero();
  hPrime(0, 6) = 1;
  //float QYawStd = 2.0f;

  if (abs(z(0) - zFromX(0)) > PI) {
    if (z(0) < zFromX(0))
        z(0) += 2.f * PI;
  }else{
        z(0) -= 2.f * PI;
  }
  update_ekf(z, hPrime, R_Mag, zFromX, dt);
  estAttitude(0) = ekfState(6);
  xt_at = Euler3212EP(estAttitude);
}

void EKF::updateFromGps(Vector3f pos, Vector3f vel, float dt) {

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

void EKF::getAttitude(quat_t *quat, attitude_t *att){
  unsigned long currentTime = micros();
  
  quat->w = xt_at(0);
  quat->x = xt_at(1);
  quat->y = xt_at(2);
  quat->z = xt_at(3);
  
  quat->dt = (currentTime >= quat->t) ? (currentTime - quat->t) / 1000000.0f : (currentTime + (ULONG_MAX - quat->t + 1)) / 1000000.0f;
  quat->t = currentTime;

  att->yaw = estAttitude(0);
  att->pitch = estAttitude(1);
  att->roll = estAttitude(2);
  att->t = currentTime;

}

void EKF::getPosVel(vec_t *pos, vec_t *vel){
  unsigned long currentTime = micros();
  
  pos->x = ekfState(0);
  pos->y = ekfState(1);
  pos->z = ekfState(2);
  pos->t = currentTime;

  vel->x = ekfState(3);
  vel->y = ekfState(4);
  vel->z = ekfState(5);
  vel->t = currentTime;

}

float EKF::yawFromMag(vec_t mag, quat_t quat) {

  VectorXf quat_readings(4);
  quat_readings(0) = quat.w;
  quat_readings(1) = quat.x;
  quat_readings(2) = quat.y;
  quat_readings(3) = quat.z;
  
  VectorXf euler_angles = EPEuler321(quat_readings);
  float pitch = euler_angles(1);
  float roll = euler_angles(2);

  float Bx = mag.x;
  float By = mag.y;

  float yawMag = atan2f(By * cos(roll) - Bx * sin(roll), Bx * cos(pitch) + By * sin(pitch) * sin(roll));

  return yawMag;
}
/*
float QuadEstimatorEKF::zFromBar(float P) {
  float P0 = 1013.0; //hPa (hectopascal) -> 1hPa = 1mBar
  float altitude_from_bar = 44330 * (1 - pow(P/P0, 1/(5.255)));
  return altitude_from_bar;
}
*/
void EKF::updateFromBar(float altitude, float dt) {
   VectorXf z(1), zFromX(1);
   z(0) = altitude;
   zFromX(0) = ekfState(2);

   MatrixXf hprime(1, 7);
   hprime.setZero();
   hprime(0, 2) = 1;

   update_ekf(z, hprime, R_bar, zFromX, dt);  
}
