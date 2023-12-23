#include "QuadEstimatorEKF.h"
#include<iostream>
using namespace Eigen;
using Eigen::VectorXf;
using Eigen::MatrixXf;


VectorXf ini_state = VectorXf::Zero(7);
VectorXf ini_stdDevs = VectorXf::Ones(7);
QuadEstimatorEKF estimator = QuadEstimatorEKF(ini_state, ini_stdDevs, 0, 0, 0, 0, 0, 0);
double dt = 0.01;

Vector4f quat(0.981163464957431, 0.0432127886478935, -0.158131735269420, 0.102202075471507); 

Vector3f angvel(0.131454436246810, 0.0149125032680754, 0.400393801802833);
Vector3f est_angvel;
Vector3f est_pqr;
Vector4f quat_2(0.980941450090188, 0.0441788987155188, -0.158035627924771, 0.104052086522407);
Vector3f euler;
Vector3f euler_2;
Vector4f quat_attitude_2(4);
Vector3f imu_acc;
Vector3f inertial_acc;
MatrixXf R_bg(3, 3);


Vector3f EulerVelocities_to_BodyRates(Vector3f omega){ //roll pitch yaw to p q r 
      Matrix3f m;
      m(0, 0) = 1;
      m(1, 0) = 0;
      m(2, 0) = 0;
      m(0, 1) = sin(euler(0)) * tan(euler(1));
      m(0, 2) = cos(euler(0)) * tan(euler(1));
      m(1, 1) = cos(euler(0));
      m(1, 2) = -sin(euler(0));
      m(2, 1) = sin(euler(0)) / cos(euler(1));
      m(2, 2) = cos(euler(0)) / cos(euler(1));
      m =  m.inverse().eval();
      return m*omega;
}

Vector3f ToEulerAngles(Vector4f q) { //roll pitch yaw in 321 sequence
    Vector3f angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q(0) * q(2) - q(1) * q(3)));
    double cosp = std::sqrt(1 - 2 * (q(0) * q(2) - q(1) * q(3)));
    angles(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

int main(){
    /*
    while(1){
        
        std::cout << "quat: ";
        std::cin >> quat(0) >> quat(1) >> quat(2) >> quat(3);
        std::cout << "imu_acc: ";
        std::cin >> imu_acc(0) >> imu_acc(1) >> imu_acc(2);
        R_bg = estimator.quatRotMat_2(quat).transpose();
        inertial_acc = R_bg * imu_acc;
        inertial_acc(2) = inertial_acc(2) - 9.81; //remove gravity

        std::cout << "inertial acc: " << inertial_acc << std::endl;
    }
    */
    std::cout << "quat: ";
    std::cin >> quat(0) >> quat(1) >> quat(2) >> quat(3);
    std::cout << estimator.quatRotMat(quat);    
    
    
        /*
        std::cout << "quat: ";
        std::cin >> quat(0) >> quat(1) >> quat(2) >> quat(3);   
        std::cout << "angvel: ";
        std::cin >> angvel(0) >> angvel(1) >> angvel(2);
        std::cout << "quat_2: ";
        std::cin >> quat_2(0) >> quat_2(1) >> quat_2(2) >> quat_2(3);
        
        euler = ToEulerAngles(quat);
        euler_2 = ToEulerAngles(quat_2);
        est_angvel = (euler_2 - euler)/0.01;
        est_pqr = EulerVelocities_to_BodyRates(est_angvel); 
        std::cout << "roll, pitch, yaw before integrating" << euler << "\n";
        std::cout << "roll, pitch, yaw after integrating" << euler_2 << "\n";

        
        std::cout << "estimated angvel roll pitch yaw" << est_angvel << "\n";
        std::cout << "estimated pqr roll pitch yaw" << est_pqr << "\n";
        std::cout << "real angvel" << angvel << "\n";
        //std::cout << "real yaw, pitch, roll after integrating" << estimator.EPEuler123(quat_2) << "\n";
    
    
    /*
    while(1){
        std::cout << "quat: ";
        std::cin >> quat(0) >> quat(1) >> quat(2) >> quat(3);   
        std::cout << "angvel: ";
        std::cin >> angvel(0) >> angvel(1) >> angvel(2);
        std::cout << "quat_2: ";
        std::cin >> quat_2(0) >> quat_2(1) >> quat_2(2) >> quat_2(3);
        
        
        euler_attitude = estimator.EPEuler123(quat);
        Vector3f gyro = angvel;
        MatrixXf A(4,4);
        MatrixXf B(4,4);

        B << 0, gyro.z(), -gyro.y(), gyro.x(),
        -gyro.z(), 0, gyro.x(), gyro.y(),
        gyro.y(), -gyro.x(), 0, gyro.z(),
        -gyro.x(), -gyro.y(), -gyro.z(), 0;

        A = A.setIdentity() + 0.01 * .5f * B;

        quat_attitude_2 = A*quat;
        euler_attitude_2 = estimator.EPEuler123(quat_attitude_2);

        std::cout << "yaw, pitch, roll before integrating" << euler_attitude << "\n";
        std::cout << "estimated yaw, pitch, roll after integrating" << euler_attitude_2 << "\n";
        std::cout << "real yaw, pitch, roll after integrating" << estimator.EPEuler123(quat_2) << "\n";
    }
    */
}