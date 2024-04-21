close all
imu = imuSensor("accel-gyro-mag");
imu.Magnetometer.RandomWalk = [0.1,0.1,0.1];
imu.Magnetometer.NoiseDensity = [0.0125,0.0125,0.0125];
FUSE = ahrsfilter;
orientation = zeros(3, 1000);
yaw = zeros(1,1000);
true_yaw = 0;

for i=1:1000
   true_yaw = true_yaw + 0.01;
   q = eul2quat([true_yaw, 0, 0], "ZYX");
   [accelReadings,gyroReadings,magReadings] = imu([0,0,0],[0,0,0],QuatRotMat(q));
   display(magReadings);
   yaw(i) = atan2(magReadings(2), magReadings(1));
   orientation(:,i) = quat2eul(FUSE(accelReadings,gyroReadings,magReadings), "ZYX");
   
end
plot(orientation(1,:)')
hold on
plot(yaw');

%display(var(180/pi*orientation(1,:)));