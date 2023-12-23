#include "mex.hpp"
#include "mexAdapter.hpp"
#include "QuadEstimatorEKF.h"

using namespace Eigen;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace matlab::data;

VectorXf temp_omega(3);
VectorXf temp_q(4);
VectorXf ini_state = VectorXf::Zero(7);
VectorXf ini_stdDevs = VectorXf::Ones(7);
QuadEstimatorEKF estimator = QuadEstimatorEKF(ini_state, ini_stdDevs, 0, 0, 0, 0, 0, 0);
double dt = 0.01;

bool use_kf_attitude = 1;

class MexFunction : public matlab::mex::Function {
        ArrayFactory factory;
public:
    void get_ekfState(matlab::mex::ArgumentList outputs){
        TypedArray<double> pos_fb = factory.createArray<double>({1,3});
        TypedArray<double> vel_fb = factory.createArray<double>({1,3});
        TypedArray<double> q_fb = factory.createArray<double>({1,4});
        TypedArray<double> acceleration_inertial = factory.createArray<double>({1,3});

        pos_fb[0] = estimator.ekfState(0);
        pos_fb[1] = estimator.ekfState(1);
        pos_fb[2] = estimator.ekfState(2);
        vel_fb[0] = estimator.ekfState(3);
        vel_fb[1] = estimator.ekfState(4);
        vel_fb[2] = estimator.ekfState(5);
        q_fb[0] = estimator.xt_at[0];
        q_fb[1] = estimator.xt_at[1];
        q_fb[2] = estimator.xt_at[2];
        q_fb[3] = estimator.xt_at[3];
        acceleration_inertial[0] = estimator.inertial_accel(0);
        acceleration_inertial[1] = estimator.inertial_accel(1);
        acceleration_inertial[2] = estimator.inertial_accel(2);

        outputs[0] = pos_fb; 
        outputs[1] = vel_fb; 
        outputs[2] = q_fb;
        outputs[3] = acceleration_inertial;
    }
    
    void get_ekfCov(matlab::mex::ArgumentList outputs){
        
        TypedArray<float> data = factory.createArray<float>({7, 7});
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                data[i][j] = estimator.ekfCov(i, j);
            }
        }
        outputs[0] = data;
    }

    void updateFromGps(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs){
        Vector3f gps_pos = Vector3f(inputs[1][0], inputs[1][1], inputs[1][2]);
        Vector3f gps_vel = Vector3f(inputs[2][0], inputs[2][1], inputs[2][2]);
        estimator.updateFromGps(gps_pos, gps_vel, dt);
    }
    void predict(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs){
        
        TypedArray<double> pos_fb = factory.createArray<double>({1,3});
        TypedArray<double> vel_fb = factory.createArray<double>({1,3});
        TypedArray<double> q_fb = factory.createArray<double>({1,4});
        TypedArray<double> omega_fb = factory.createArray<double>({1,3});
        TypedArray<double> acceleration_inertial = factory.createArray<double>({1,3});
        estimator.acc = Vector3f(inputs[1][0], inputs[1][1], inputs[1][2]);
        estimator.gyro = Vector3f(inputs[2][0], inputs[2][1], inputs[2][2]);

        temp_omega = estimator.gyro; //should be in the order roll, pitch, yaw but needs to be checked
        omega_fb[0] = temp_omega(0);
        omega_fb[1] = temp_omega(1);
        omega_fb[2] = temp_omega(2);

        estimator.ut(0) = estimator.acc(0);  //  control vector
        estimator.ut(1) = estimator.acc(1);
        estimator.ut(2) = estimator.acc(2);
        estimator.ut(3) = temp_omega(2);
        
        if(use_kf_attitude)
            estimator.kf_attitudeEstimation(dt);
        else
            estimator.complimentary_filter_attitude_estimation(dt);
        estimator.predict_2(dt);
        //estimator.attitude_integration(dt);

        pos_fb[0] = estimator.ekfState(0);
        pos_fb[1] = estimator.ekfState(1);
        pos_fb[2] = estimator.ekfState(2);
        vel_fb[0] = estimator.ekfState(3);
        vel_fb[1] = estimator.ekfState(4);
        vel_fb[2] = estimator.ekfState(5);
        
        if(use_kf_attitude){
            q_fb[0] = estimator.xt_at[0];
            q_fb[1] = estimator.xt_at[1];
            q_fb[2] = estimator.xt_at[2];
            q_fb[3] = estimator.xt_at[3];
        }
        else{
            temp_q = estimator.Euler1232EP(estimator.estAttitude); //check angle and quaternion conventions !!
            q_fb[0] = temp_q(0);
            q_fb[1] = temp_q(1);
            q_fb[2] = temp_q(2);
            q_fb[3] = temp_q(3);
        }
        
        acceleration_inertial[0] = estimator.inertial_accel(0);
        acceleration_inertial[1] = estimator.inertial_accel(1);
        acceleration_inertial[2] = estimator.inertial_accel(2);
        
        outputs[0] = pos_fb; 
        outputs[1] = vel_fb; 
        outputs[2] = q_fb; 
        outputs[3] = omega_fb;
        //outputs[4] = acceleration_inertial;
    
    }
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
       std::string function_name = matlab::engine::convertUTF16StringToUTF8String(inputs[0][0]);; 
       if(function_name == "predict"){
           predict(outputs, inputs);
       }
       else if (function_name == "updateFromGps"){
            updateFromGps(outputs,inputs);
       }
       else if (function_name == "get_ekfState"){
           get_ekfState(outputs);
       }
       else if(function_name == "get_ekfCov"){
           get_ekfCov(outputs);
       }
    }
    
};