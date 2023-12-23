clear all
gt_attitude = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/attitude");
gt_acc = h5read("sensor_records.hdf5", "/trajectory_0000/groundtruth/acceleration");
imu_acc = h5read("sensor_records.hdf5", "/trajectory_0000/imu/accelerometer");
imu_gyro = h5read("sensor_records.hdf5", "/trajectory_0000/imu/gyroscope");

world_acc = zeros(3, 8818);
gt_pitch = zeros(1,8818);
gt_roll = zeros(1,8818);
gt_yaw = zeros(1,8818);
accelPitch = zeros(1,8818);
accelRoll = zeros(1,8818);

q = zeros(4,8818);
q(:,1) = [1,0,0,0];
dt = 0.01;
attitudeTau = 0.001;
eye = eye(4);

for ii = 2:length(imu_acc)
    %q = gt_attitude(:,ii);
    %M = [1 - 2*q(3)*q(3) - 2*q(4)*q(4), 2*q(2)*q(3) - 2*q(1)*q(4), 2*q(2)*q(4) + 2*q(1)*q(3);
    %       2*q(2)*q(3) + 2*q(1)*q(4),  1 - 2*q(2)*q(2) - 2*q(4)*q(4), 2*q(3)*q(4) - 2*q(1)*q(2);
    %      2*q(2)*q(4) - 2*q(1)*q(3), 2*q(3)*q(4) + 2*q(1)*q(2), 1 - 2*q(2)*q(2) - 2*q(3)*q(3)];
    %world_acc(:,ii) = M*imu_acc(:,ii) + [0,0,9.81]';
    gt_angles = euler(quaternion(gt_attitude(:,ii)'), "zyx", "frame");
    gt_yaw(ii) = gt_angles(1); 
    gt_pitch(ii) = gt_angles(2); 
    gt_roll(ii) = gt_angles(3);
    gyro = imu_gyro(:,ii-1);
    B =  [0, -gyro(1), -gyro(2), -gyro(3);
      gyro(1), 0, gyro(3), -gyro(2);
      gyro(2), -gyro(3), 0, gyro(1);
      gyro(3), gyro(2), -gyro(1), 0];
    q(:,ii) = (eye + 1/2 * dt * B)*q(:,ii-1);
    
    predictedAttitude = quat2eul(quaternion(q(:,ii)'), "zyx");

    accelPitch(ii) = asin(-imu_acc(1,ii-1) / (-9.81));
    accelRoll(ii) = asin(imu_acc(2,ii-1) / (-9.81 * cos(predictedAttitude(2)) ) );
    
    estPitch = attitudeTau / (attitudeTau + dt) * (predictedAttitude(3))+dt / (attitudeTau + dt) * accelRoll(ii);
    estRoll =  attitudeTau / (attitudeTau + dt) * (predictedAttitude(2))+dt / (attitudeTau + dt) * accelPitch(ii);
    q(:,ii) = eul2quat([predictedAttitude(1), estPitch, estRoll]);

end



%% plot angle errors
%{
acc_err = gt_acc - world_acc;

roll_err = gt_roll - accelRoll;
pitch_err = gt_pitch - accelPitch;
figure(1)
subplot(2, 1, 1)
plot(rad2deg(pitch_err));
title("pitch error");

subplot(2, 1, 2)
plot(rad2deg(roll_err));
title("roll error");


figure(2)
subplot(2, 1, 1)
plot(rad2deg([gt_pitch;accelPitch]'));
title("pitch 1");

subplot(2, 1, 2)
plot(rad2deg([gt_roll; accelRoll]'));
title("roll 1");

figure(3)
subplot(2, 1, 1)
plot(rad2deg([gt_pitch;accelPitch_2]'));
title("pitch 2");

subplot(2, 1, 2)
plot(rad2deg([gt_roll; accelRoll_2]'));
title("roll 2");
%}