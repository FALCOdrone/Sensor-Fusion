clc
clear all
close all

gps_pos_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/position");
gps_HDOP_00 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/HDOP");
gps_VDOP_00 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/VDOP"); %noi non dovremmo averlo
gps_vel_000 = h5read("sensor_records.hdf5", "/trajectory_0000/gps/velocity");
acc_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/accelerometer");
gyro_000 = h5read("sensor_records.hdf5", "/trajectory_0000/imu/gyroscope");

%h5disp("sensor_records.hdf5", "/trajectory_0000/groundtruth")
gt_pos = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/position");
gt_vel = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/velocity");
gt_acc = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/acceleration");
gt_attitude = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/attitude");

imu_acc_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/accelerometer","init_bias_est"); 
imu_gyro_bias = h5readatt("sensor_records.hdf5","/trajectory_0000/imu/gyroscope","init_bias_est"); 
gps_pos_bias = gps_pos_000(:,1); % sembra che ci sia un bias soprattutto lunzo z

%% simulation with Estimator class

ekf = Estimator([0,0,0,0,0,0,0]', eye(7), [1,0,0,0]', imu_acc_bias, imu_gyro_bias);
ekf_quat = zeros(4,length(gyro_000));
ekf_quat(1,1) = 1;
ekf_pos = zeros(3,length(gyro_000));
ekf_attitude_imu_residual = zeros(4, length(gyro_000));
ekf_gps_residual = zeros(6, length(gyro_000));
%if you want fading memory or residual method set the corresponding variable =1
fading = 0;
residual=1;
jj = 1;
for ii = 1:length(gyro_000)
  gyro = gyro_000(:, ii);
  acc = acc_000(:,ii);
  
  ekf.predict(acc,gyro,fading,residual);
  
  % gps update
  if(mod(ii, 100) == 1 && jj <= length(gps_pos_000)) 
        ekf.updateFromGps(gps_pos_000(:,jj) - gps_pos_bias, gps_vel_000(:,jj),gps_HDOP_00(1,jj),gps_VDOP_00(1,jj),residual);
        jj = jj + 1;
  end
  ekf_pos(:,ii) = ekf.ekfState(1:3);
  ekf_vel(:,ii) = ekf.ekfState(4:6);
  ekf_quat(:,ii) = ekf.xt_at;
  ekf_attitude_imu_residual(:,ii) = ekf.attitude_imu_residual;
  ekf_gps_residual(:,ii) = ekf.gps_residual;
end

%% X-Y trajectory
%temporal axis
time=zeros(1, length(gyro_000));
for i=1:length(gyro_000)
    time(i)=(i-1)/100;
end
time2=zeros(1,length(gps_pos_000));
for i=1:length(gps_pos_000)
    time2(i)=(i-1)/1;
end

figure(7)
scatter(gt_pos(1,:)', gt_pos(2,:)') %real trajectory
hold on
plot(ekf_pos(1,:)',gt_pos(2,:)',linewidth=2.0) %estimated trajectory
title("trajectory")
hold off

%comparison between gps data, estimated position and ground position
%x
figure(8)   
scatter(time2, gps_pos_000(1,:)-gps_pos_bias(1,1), ekf.R_GPS(1,1),"blue" ) %real trajectory
hold on
plot(time, gt_pos(1,:),"green") %estimated trajectory
plot(time, ekf_pos(1,:),"red")
legend("GPS ", "ground-t", "ekf")
title("x")
hold off

%y
figure(9)   
scatter(time2, gps_pos_000(2,:)-gps_pos_bias(2,1), ekf.R_GPS(2,2),"blue" ) %real trajectory
hold on
plot(time, gt_pos(2,:),"green") %estimated trajectory
plot(time, ekf_pos(2,:),"red")
legend("GPS ", "ground-t", "ekf")
title("y")
hold off

%z
figure(10)   
scatter(time2, gps_pos_000(3,:)-gps_pos_bias(3,1), ekf.R_GPS(3,3),"blue" ) %real trajectory
hold on
plot(time, gt_pos(3,:),"green") %estimated trajectory
plot(time, ekf_pos(3,:),"red")
legend("GPS ", "ground-t", "ekf")
title("z")
hold off



%% ERROR PLOT
% ATTITUDE error plot
figure(1)
subplot(3,1,1)

plot((gt_attitude - ekf_quat)');
title("Error attitude");
legend("true q0", "true qx", "true qy", "true qz");

% position error plot
subplot(3,1,2)
pos_err = gt_pos - ekf_pos;
plot(pos_err');
title("position error");
legend("error x", "error y", "error z");
ylabel("m")

% velocity error plot
subplot(3,1,3)
vel_err = gt_vel - ekf_vel;
plot(vel_err');
title("velocity error");
legend("error vx", "error vy", "error vz");
ylabel("m/s")

%% normalized residuals plot for Bar-shalom Adjustable Process Noise

figure(4)
subplot(2,1,1)
plot(ekf_gps_residual(1,:)');
subplot(2,1,2)
plot(ekf_attitude_imu_residual');


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

% ATTITUDE ERROR 
attitude_error = gt_attitude - q;

figure(4)
plot(attitude_error');
title("attitude error");
legend("err q0", "err qx", "err qy", "err qz");

% World Acceleration Error
figure(5)
acc_error = gt_acc - inertial_acc;
plot(acc_error');
title("acceleration error");
legend("error x", "error y", "error z");

%position plot
figure(2) 
subplot(2, 1, 1)
plot(gt_pos');
title("ref pos");
legend("x", "y", "z");

subplot(2, 1, 2)
plot(pos');
title("kf pos");
legend("x", "y", "z");
