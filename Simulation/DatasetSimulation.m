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

%simulated magnetometer setup
imu = imuSensor("accel-gyro-mag");
imu.Magnetometer.RandomWalk = [0.1,0.1,0.1];
imu.Magnetometer.NoiseDensity = [0.0125,0.0125,0.0125];
%% sensor variance estimation
gt_GPS_pos=zeros(3,length(gps_pos_000(1,:)));
for i=1:length(gps_pos_000)
    gt_GPS_pos(:,i)=gt_pos(:,(i-1)*100+1);
end
errorpos_gps= gps_pos_000 - gt_GPS_pos;

gt_GPS_vel=zeros(3,length(gps_vel_000(1,:)));
for i=1:length(gps_vel_000)
    gt_GPS_vel(:,i)=gt_vel(:,(i-1)*100+1);
end
errorvel_gps= gps_vel_000 - gt_GPS_vel;


var_gps=zeros(6,1);
bias_gps=zeros(6,1);

bias_gps(1,1) = mean(errorpos_gps(1,:));
bias_gps(2,1) = mean(errorpos_gps(2,:));
bias_gps(3,1) = mean(errorpos_gps(3,:));
bias_gps(4,1) = mean(errorvel_gps(1,:));
bias_gps(5,1) = mean(errorvel_gps(2,:));
bias_gps(6,1) = mean(errorvel_gps(3,:));

var_gps(1,1) = var(errorpos_gps(1,:));
var_gps(2,1) = var(errorpos_gps(2,:));
var_gps(3,1) = var(errorpos_gps(3,:));
var_gps(4,1) = var(errorvel_gps(1,:));
var_gps(5,1) = var(errorvel_gps(2,:));
var_gps(6,1) = var(errorvel_gps(3,:));
gps_pos_bias(1,1)=bias_gps(1,1);
gps_pos_bias(2,1)=bias_gps(2,1);
gps_pos_bias(3,1)=bias_gps(3,1);

%% sensor variance estimation 1
clear all
gps_pos_001 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/position");
gt_pos1 = h5read("sensor_records.hdf5", "/trajectory_0001/groundtruth/position");
gps_HDOP_01 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/HDOP");
gps_VDOP_01 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/VDOP");

gt_GPS_pos1=zeros(3,length(gps_pos_001(1,:)));
for i=1:length(gps_pos_001)
    gt_GPS_pos1(:,i)=gt_pos1(:,(i-1)*100+1);
end
errorpos_gps1= gps_pos_001- gt_GPS_pos1;

var_gps1 = var(errorpos_gps1')';

errorpos_gpsNorm1=zeros(3,length(gps_pos_001(1,:)));
errorpos_gpsNorm1(1,:)=abs(errorpos_gps1(1,:))./gps_HDOP_01;
errorpos_gpsNorm1(2,:)=abs(errorpos_gps1(2,:))./gps_HDOP_01;
errorpos_gpsNorm1(3,:)=abs(errorpos_gps1(3,:))./gps_VDOP_01;

var_coeff1 = var(errorpos_gpsNorm1')';
mean_coeff1 = mean(errorpos_gpsNorm1')';

%% sensor variance estimation 2
gps_pos_002 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/position");
gt_pos2 = h5read("sensor_records.hdf5", "/trajectory_0002/groundtruth/position");
gt_pos2 = h5read("sensor_records.hdf5", "/trajectory_0002/groundtruth/position");
gps_HDOP_02 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/HDOP");
gps_VDOP_02 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/VDOP");

gt_GPS_pos2=zeros(3,length(gps_pos_002(1,:)));
for i=1:length(gps_pos_002)
    gt_GPS_pos2(:,i)=gt_pos2(:,(i-1)*100+1);
end
errorpos_gps2= gps_pos_002- gt_GPS_pos2;

var_gps2 = var(errorpos_gps2')';

errorpos_gpsNorm2=zeros(3,length(gps_pos_002(1,:)));
errorpos_gpsNorm2(1,:)=abs(errorpos_gps2(1,:))./gps_HDOP_02;
errorpos_gpsNorm2(2,:)=abs(errorpos_gps2(2,:))./gps_HDOP_02;
errorpos_gpsNorm2(3,:)=abs(errorpos_gps2(3,:))./gps_VDOP_02;

var_coeff2 = var(errorpos_gpsNorm2')';
mean_coeff2 = mean(errorpos_gpsNorm2')';

%% sensor variance estimation 3
gps_pos_003 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/position");
gt_pos3 = h5read("sensor_records.hdf5", "/trajectory_0003/groundtruth/position");
gps_HDOP_03 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/HDOP");
gps_VDOP_03 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/VDOP");

gt_GPS_pos3=zeros(3,length(gps_pos_003(1,:)));
for i=1:length(gps_pos_003)
    gt_GPS_pos3(:,i)=gt_pos3(:,(i-1)*100+1);
end
errorpos_gps3= gps_pos_003- gt_GPS_pos3;

errorpos_gpsNorm3=zeros(3,length(gps_pos_003(1,:)));
errorpos_gpsNorm3(1,:)=abs(errorpos_gps3(1,:))./gps_HDOP_03;
errorpos_gpsNorm3(2,:)=abs(errorpos_gps3(2,:))./gps_HDOP_03;
errorpos_gpsNorm3(3,:)=abs(errorpos_gps3(3,:))./gps_VDOP_03;

var_gps3=zeros(6,1);
var_gps3(1,1) = var(errorpos_gps3(1,:));
var_gps3(2,1) = var(errorpos_gps3(2,:));
var_gps3(3,1) = var(errorpos_gps3(3,:));

var_coeff3 = var(errorpos_gpsNorm3')';
mean_coeff3 = mean(errorpos_gpsNorm3')';

%% simulation with Estimator class

ekf = Estimator([0,0,0,0,0,0,0]', eye(7), [1,0,0,0]', imu_acc_bias, imu_gyro_bias);
ekf_quat = zeros(4,length(gyro_000));
ekf_quat(1,1) = 1;
ekf_pos = zeros(3,length(gyro_000));
ekf_vel = zeros(3,length(gyro_000));
ekf_attitude_imu_residual = zeros(4, length(gyro_000));
ekf_gps_residual = zeros(6, length(gyro_000));
yaw_reading = zeros(1, length(gyro_000));
%if you want fading memory or residual method set the corresponding variable =1
fading = 0;
residual=0;
jj = 1;
for ii = 1:length(gyro_000)
  gyro = gyro_000(:, ii);
  acc = acc_000(:,ii);
  
  ekf.predict(acc,gyro,fading,residual);

  %simulated magnetometer update (supposing mag frequency equal that of IMU)
  [~,~,magReadings] = imu([0,0,0],[0,0,0],QuatRotMat(gt_attitude(:,ii)));
  yaw_reading(ii) = atan2(magReadings(2), magReadings(1));
  ekf.updateFromMag(yaw_reading(ii), residual);

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

%% ERROR PLOT
% ATTITUDE error plot
figure(8)
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

figure(1)
scatter(gt_pos(1,:)', gt_pos(2,:)') %real trajectory
hold on
plot(ekf_pos(1,:)',gt_pos(2,:)',linewidth=2.0) %estimated trajectory
title("trajectory")
hold off

%comparison between gps data, estimated position and ground position
%x
figure(2)   
scatter(time2, gps_pos_000(1,:)-gps_pos_bias(1,1), 1,"blue" )
hold on
plot(time, gt_pos(1,:),"green") 
plot(time, ekf_pos(1,:),"red")
legend("GPS ", "ground-t", "ekf")
title("x")
hold off

%y
figure(3)   
scatter(time2, gps_pos_000(2,:)-gps_pos_bias(2,1), 1,"blue" )
hold on
plot(time, gt_pos(2,:),"green") 
plot(time, ekf_pos(2,:),"red")
legend("GPS ", "ground-t", "ekf")
title("y")
hold off

%z
figure(4)   
scatter(time2, gps_pos_000(3,:)-gps_pos_bias(3,1), 1,"blue" ) 
hold on
plot(time, gt_pos(3,:),"green") %estimated trajectory
plot(time, ekf_pos(3,:),"red")
legend("GPS ", "ground-t", "ekf")
title("z")
hold off

%% Plot fo roll pitch and yaw
time=zeros(1, length(gyro_000));
for i=1:length(gyro_000)
    time(i)=(i-1)/100;
end
eulerZYX = quat2eul(ekf_quat','ZYX');

gt_attitude_EULER=quat2eul(gt_attitude','ZYX')';

figure(5)  
plot(time, gyro_000(1,:)-imu_gyro_bias(1,1), "blue", 'LineWidth',0.001) 
hold on
plot(time, eulerZYX(:,1)',"green") %estimated trajectory
plot(time, gt_attitude_EULER(1,:),"red")

legend("gyro","ekf_z ", "ground-z")
title("Yaw")
hold off

figure(6)
plot(time, gyro_000(2,:)-imu_gyro_bias(2,1), "blue" , 'LineWidth',0.001) 
hold on
plot(time, eulerZYX(:,2)',"green") %estimated trajectory
plot(time, gt_attitude_EULER(2,:),"red")
legend("gyro","ekf_Y ", "ground-y")
title("Pitch")
hold off

figure(7)
plot(time, gyro_000(3,:)-imu_gyro_bias(3,1),"blue" , 'LineWidth',0.001) 
hold on
plot(time, eulerZYX(:,3)',"green") %estimated trajectory
plot(time, gt_attitude_EULER(3,:),"red")
legend("gyro","ekf_x ", "ground-x")
title("Roll")
hold off



%% normalized residuals plot for Bar-shalom Adjustable Process Noise

figure(9)
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
