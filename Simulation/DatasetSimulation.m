clear all
close all

gps_pos = h5read("sensor_records.hdf5", "/trajectory_0000/gps/position");
gps_vel = h5read("sensor_records.hdf5", "/trajectory_0000/gps/velocity");
%{
GDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/GDOP");
HDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/HDOP");
PDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/PDOP");
VDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/VDOP");
%}
gps_pos_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/position");
gps_vel_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/velocity");
acc_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/accelerometer");
gyro_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/gyroscope");
gt_pos = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/position");
gt_vel = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/velocity");
gt_attitude = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/attitude");

imu_acc = h5read("sensor_records.hdf5", "/trajectory_0000/imu/accelerometer");
imu_gyro = h5read("sensor_records.hdf5", "/trajectory_0000/imu/gyroscope");
gt_pos = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/position");
gt_vel = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/velocity");
gt_attitude = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/attitude");
gt_angvel = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/angular_velocity");
gt_acc = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/acceleration");

imu_acc_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/accelerometer","init_bias_est"); 
imu_gyro_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/gyroscope","init_bias_est"); 
gps_pos_bias = gps_pos(:,1); % sembra che ci sia un bias soprattutto lunzo z

%% TRUE ATTITUDE VS ESTIMATED ATTITUDE

figure(3)
subplot(2, 1, 1)
plot(gt_attitude');
title("true attitude");
legend("true q0", "true qx", "true qy", "true qz");

subplot(2, 1, 2)
plot(q');
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
estQuat(1, :) = 1;

dt = 0.01;

for ii = 2:length(gyro_000)
    gyro = gyro_000(:, ii - 1) - imu_gyro_bias;
    acc = acc_000(:,ii-1) - imu_acc_bias;
    quatToEuler = quat2eul(estQuat(:, ii - 1)', 'ZYX'); % seqence of yaw, pitch and roll
    accelPitch = asin(-acc(1) / (-9.81));
    accelRoll = asin(acc(2) / (-9.81 * cos(accelPitch)));
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

%% position
%reused estQuat from previous section, attitude estimation is independent from position 
q = estQuat;
ekfCov = eye(7);

Q = eye(7)*0.5^2;   %model covariance
Q(7,7) = 0.095^2;   %related to yaw
Q = Q * dt;

R_gps = zeros(6,6); %GPS noise matrix
R_gps(1,1) = 0.1^2;
R_gps(2,2) = 0.1^2;
R_gps(3,3) = 0.3^2;
R_gps(4,4) = 0.1^2;
R_gps(5,5) = 0.1^2;
R_gps(6,6) = 0.3^2;


pos_err = zeros(3,8818);
attitude_error = zeros(4, 8818);
pos = zeros(3, 8818);
vel = zeros(3, 8818);
inertial_acc = zeros(3, 8818);

predicted_state = zeros(6,8818);
jj = 1;
for ii = 2:length(gyro_000)
  gyro = gyro_000(:, ii - 1) - imu_gyro_bias;
  acc = acc_000(:,ii-1) - imu_acc_bias;
  
  %imu predict
  M = QuatRotMat(q(:,ii) );
  inertial_acc(:,ii) = M*acc;
  inertial_acc(3,ii) = inertial_acc(3,ii) + 9.81;
  pos(:,ii) = pos(:,ii-1) + vel(:,ii-1)*dt;
  vel(:,ii) = vel(:,ii-1) + inertial_acc(:,ii) * dt;
  
 
  estAttitude(ii,:) = quat2eul(estQuat(:, ii)', 'ZYX');
  RbgPrime = zeros(3,3);
  RbgPrime(1, 1) = -cos(estAttitude(ii,2)) * sin(estAttitude(ii,1));
  RbgPrime(1, 2) = -sin(estAttitude(ii,3)) * sin(estAttitude(ii,2)) * sin(estAttitude(ii,1)) - cos(estAttitude(ii,3)) * cos(estAttitude(ii,1));
  RbgPrime(1, 3) = -cos(estAttitude(ii,3)) * sin(estAttitude(ii,2)) * sin(estAttitude(ii,1)) + sin(estAttitude(ii,3)) * cos(estAttitude(ii,1));
  RbgPrime(2, 1) = cos(estAttitude(ii,2)) * cos(estAttitude(ii,1));
  RbgPrime(2, 2) = sin(estAttitude(ii,3)) * sin(estAttitude(ii,2)) * cos(estAttitude(ii,1)) - cos(estAttitude(ii,3)) * sin(estAttitude(ii,1));
  RbgPrime(2, 3) = cos(estAttitude(ii,3)) * sin(estAttitude(ii,2)) * cos(estAttitude(ii,1)) + sin(estAttitude(ii,3)) * sin(estAttitude(ii,1));
    
  gPrime = eye(7);

  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = dt;

  helper_matrix = RbgPrime * acc;
  gPrime(4, 7) = helper_matrix(1) * dt;
  gPrime(5, 7) = helper_matrix(2) * dt;
  gPrime(6, 7) = helper_matrix(3) * dt;
  ekfCov = gPrime * ekfCov * gPrime' + Q;
  
  % gps update
  if(mod(ii, 100) == 1 && jj <= length(gps_pos_000)) 
        hprime = eye(6,7);
        z = [gps_pos_000(:,jj) - gps_pos_bias; gps_vel_000(:,jj)];
        zFromX = [pos(:,ii); vel(:,ii)]; % concatenate pos and vel

        toInvert = hprime*ekfCov*hprime' + R_gps;
        K = ekfCov * hprime' / toInvert;

        updated_state = [pos(:,ii); vel(:,ii); 0] + K*(z - zFromX); 
        pos(:,ii) = updated_state(1:3);
        vel(:,ii) = updated_state(4:6);

        ekfCov = (eye(7) - K*hprime)*ekfCov;

        jj = jj + 1;
  end

end
%% position with ekf_mex
pos_err = zeros(3,8818);
attitude_error = zeros(4, 8818);
pos = zeros(3, 8818);
vel = zeros(3, 8818);
q = zeros(4, 8818);
omega = zeros(3, 8818);
inertial_acc = zeros(3, 8818);
jj = 1;
for ii = 1:length(imu_gyro)
    ekf_mex("predict", imu_acc(:,ii), imu_gyro(:,ii));
    
    if(mod(ii, 100) == 1 && jj <= length(gps_pos))
        ekf_mex("updateFromGps", (gps_pos(:,jj) - gps_pos_bias), gps_vel(:,jj));
        jj = jj + 1;
    end
    
    [pos(:,ii), vel(:,ii), q(:,ii), inertial_acc(:,ii)] = ekf_mex("get_ekfState");
end
%% position plot
figure(1)
pos_err = gt_pos - pos;
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
