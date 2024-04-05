clear all
close all

gps_pos_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/position");
gps_vel_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/velocity");
acc_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/accelerometer");
gyro_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/gyroscope");
gt_pos = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/position");
gt_vel = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/velocity");
gt_attitude = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/attitude");

imu_acc_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/accelerometer","init_bias_est"); 
imu_gyro_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/gyroscope","init_bias_est"); 
gps_pos_bias = gps_pos_000(:,1); % sembra che ci sia un bias soprattutto lunzo z

%% TRUE ATTITUDE VS ESTIMATED ATTITUDE

figure(3)
subplot(2, 1, 1)
plot((gt_attitude - ekf_quat)');
title("true attitude");
legend("true q0", "true qx", "true qy", "true qz");

subplot(2, 1, 2)
plot(ekf_quat');
title("estimated attitude");
legend("kf q0", "kf qx", "kf qy", "kf qz");
%% ATTITUDE ERROR
 
attitude_error = gt_attitude - q;

figure(4)
plot(attitude_error');
title("attitude error");
legend("err q0", "err qx", "err qy", "err qz");
%% World Acceleration Error

figure(5)
acc_error = gt_acc - inertial_acc;
plot(acc_error');
title("acceleration error");
legend("error x", "error y", "error z");

%% TESTING ATTITUDE ESTIMATION WITH KALMAN FILTER
% Prova commento github
%import functions.fromQuatToEuler.*
estAttitude = zeros(length(gyro_000), 3);
ekfCov_at = eye(4);
Q_at = eye(4) * 0.0001; % needs to be settled correctly consulting the datasheet
H_at = eye(4); 
R_at = eye(4) * 100;
estQuat = zeros(4, length(gyro_000));
accellData = zeros(2, length(gyro_000));
estQuat(1, :) = 1;

dt = 0.01;



for ii = 2:length(gyro_000)
    gyro = gyro_000(:, ii - 1) - imu_gyro_bias;
    acc = acc_000(:,ii-1) - imu_acc_bias;
    quatToEuler = quat2eul(estQuat(:, ii - 1)', 'ZYX'); % seqence of yaw, pitch and roll
    accelPitch = asin(-acc(1) / (-9.81));
    accelRoll = asin(acc(2) / (-9.81 * cos(accelPitch)));

    accellData(1, ii-1) = accelPitch;
    accellData(2, ii-1) = accelRoll;

    estAttitude(ii - 1, 1) = quatToEuler(1);

    B =  [0, -gyro(1), -gyro(2), -gyro(3);
      gyro(1), 0, -gyro(3), gyro(2);
      gyro(2), gyro(3), 0, -gyro(1);
      gyro(3), -gyro(2), gyro(1), 0];

    A = eye(4) + dt * 0.5 * B;
    estQuat(:, ii) = A * estQuat(:, ii - 1);

    % update
    ekfCov_pred = A * ekfCov_at * A' + Q_at;
    M = H_at * ekfCov_pred * H_at' + R_at;
    k_at = ekfCov_pred * H_at'/M;
    
    euler = [estAttitude(ii - 1, 1), accelPitch, accelRoll];
    z = eul2quat(euler, 'ZYX');
    estQuat(:, ii) = estQuat(:, ii) + k_at * (z' - H_at * estQuat(:, ii));
    ekfCov_at = ekfCov_pred - k_at * H_at * ekfCov_pred;
    
    
end

figure(1)
subplot(2, 1, 1)
plot(estQuat');
title("estimated attitude");
legend("est q0", "est qx", "est qy", "est qz");

subplot(2, 1, 2)
plot(gt_attitude');
title("true attitude");
legend("true q0", "true qx", "true qy", "true qz");

figure(2)
plot((gt_attitude - estQuat)');
title("error");

figure(3)
plot((gt_attitude - ekf_quat)');
title("error");

% converting estimated quaternions to euler angles and comparing it with accel
% measurements
figure(4)
subplot(2, 1, 1)
plot(rad2deg(quat2eul(estQuat', 'ZYX')));
title("estimated euler angles");
legend("yaw", "pitch", "roll");

subplot(2, 1, 2)
plot(rad2deg(accellData'));
title("accellerometer data");
legend("pitch", "roll");

%% simulation with Estimator class

ekf = Estimator([0,0,0,0,0,0,0]', eye(7), [1,0,0,0]', imu_acc_bias, imu_gyro_bias);
ekf_quat = zeros(4,length(gyro_000));
ekf_quat(1,1) = 1;
ekf_pos = zeros(3,length(gyro_000));
ekf_attitude_imu_residual = zeros(4, length(gyro_000));
ekf_gps_residual = zeros(6, length(gyro_000));

jj = 1;
for ii = 1:length(gyro_000)
  gyro = gyro_000(:, ii);
  acc = acc_000(:,ii);
  
  ekf.predict(acc,gyro);
  
  % gps update
  if(mod(ii, 100) == 1 && jj <= length(gps_pos_000)) 
        ekf.updateFromGps(gps_pos_000(:,jj) - gps_pos_bias, gps_vel_000(:,jj));
        jj = jj + 1;
  end
  ekf_pos(:,ii) = ekf.ekfState(1:3);
  ekf_quat(:,ii) = ekf.xt_at;
  ekf_attitude_imu_residual(:,ii) = ekf.attitude_imu_residual;
  ekf_gps_residual(:,ii) = ekf.gps_residual;
end
%% simulation with ekf_mex
pos_err = zeros(3,8818);
attitude_error = zeros(4, 8818);
pos = zeros(3, 8818);
vel = zeros(3, 8818);
q = zeros(4, 8818);
omega = zeros(3, 8818);
inertial_acc = zeros(3, 8818);
jj = 1;
for ii = 1:length(gyro_000)
    ekf_mex("predict", acc_000(:,ii), gyro_000(:,ii));
    
    if(mod(ii, 100) == 1 && jj <= length(gps_pos_000))
        ekf_mex("updateFromGps", (gps_pos_000(:,jj) - gps_pos_bias), gps_vel_000(:,jj));
        jj = jj + 1;
    end
    
    [pos(:,ii), vel(:,ii), q(:,ii), inertial_acc(:,ii)] = ekf_mex("get_ekfState");
end
%% position plot
figure(1)
pos_err = gt_pos - ekf_pos;
plot(pos_err');
title("position error");
legend("error x", "error y", "error z");

figure(2) 
subplot(2, 1, 1)
plot(gt_pos');
title("ref pos");
legend("x", "y", "z");

subplot(2, 1, 2)
plot(pos');
title("kf pos");
legend("x", "y", "z");

%% residuals plot

figure(1)
plot(ekf_attitude_imu_residual');
figure(2)
plot(ekf_gps_residual');