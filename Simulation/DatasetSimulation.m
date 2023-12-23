clear all

gps_pos = h5read("sensor_records.hdf5", "/trajectory_0000/gps/position");
gps_vel = h5read("sensor_records.hdf5", "/trajectory_0000/gps/velocity");
%{
GDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/GDOP");
HDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/HDOP");
PDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/PDOP");
VDOP = h5read("sensor_records.hdf5", "/trajectory_0000/gps/VDOP");
%}

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
%import functions.fromQuatToEuler.*
estAttitude = zeros(8818, 3);
estQuat = zeros(4, 8818);
estQuat(1, 1) = 1;
ekfCov_at = eye(4);
H_at = eye(4);
Q_at = eye(4) * 0.0001;
R_at = eye(4) * 0.001;
dt = 0.01;
B = zeros(4, 4);
A = zeros(4, 4);

for ii = 2:length(imu_gyro)

    gyro = imu_gyro(:, ii-1);

    quatToEuler = quat2eul(estQuat(:, ii-1)', 'ZYX');  % sequence with the order: yaw, pitch and roll
    accelPitch = asin(-imu_acc(1,ii-1) / (-9.81));
    accelRoll = asin(imu_acc(2,ii-1) / (-9.81 * cos(quatToEuler(2)) ) );
    estAttitude(ii-1, 1) = quatToEuler(1);

    B =  [0, -gyro(1), -gyro(2), -gyro(3);
      gyro(1), 0, gyro(3), -gyro(2);
      gyro(2), -gyro(3), 0, gyro(1);
      gyro(3), gyro(2), -gyro(1), 0];

    A = eye(4) + dt * 0.5 * B;

    estQuat(:, ii) = A * estQuat(:, ii - 1);

    % update
    ekfCov_pred = A * ekfCov_at * A' + Q_at;
    M = H_at * ekfCov_pred * H_at' + R_at;
    K_at = ekfCov_pred * H_at' * inv(M);
     
    euler = [estAttitude(ii-1, 1), accelPitch, accelRoll];
    z = eul2quat(euler, 'ZYX');
    estQuat(:, ii) = estQuat(:, ii) + K_at * (z' - H_at * estQuat(:, ii));
    ekfCov_at = ekfCov_pred - K_at * H_at * ekfCov_pred;
end

figure(6)
subplot(2, 1, 1)
plot(estQuat');
title("estimated attitude");
legend("est q0", "est qx", "est qy", "est qz");

subplot(2, 1, 2)
plot(gt_attitude');
title("true attitude");
legend("true q0", "true qx", "true qy", "true qz");
