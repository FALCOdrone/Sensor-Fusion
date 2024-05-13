#include "imu_old.h"

#ifdef UDOO
MPU6500 IMU;  // UDOO KEY
#else
MPU6050 IMU;
#endif

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
/*
float B_madgwick = 0.04;  // Madgwick filter parameter
float B_accel = 0.14;     // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
#ifdef HAS_MAG
float B_mag = 1.0;  // Magnetometer LP filter parameter
#endif
*/

calData calib = {0};  // Calibration data
float deadZone[3] = {0.0, 0.0, 0.0};
//Madgwick filter;

// WARNING: run this strictly when the drone is on a flat surface and not moving
void initializeImu(int calibrate) {
#ifdef UDOO
    Wire.begin(18, 21);  // UDOO KEY
#else
    Wire.begin();
#endif

    Wire.setClock(400000);  // 400khz clock

    int err = IMU.init(calib, IMU_ADDR);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true) {
            ;
        }
    }

    IMU.setGyroRange(GYRO_SCALE);
    IMU.setAccelRange(ACCEL_SCALE);

    if (calibrate) {
        Serial.println("Keep IMU level.");
        delay(5000);
        IMU.calibrateAccelGyro(&calib);
        Serial.println("Calibration done!");
        Serial.println("Accel biases X/Y/Z: ");
        Serial.print(calib.accelBias[0]);
        Serial.print(", ");
        Serial.print(calib.accelBias[1]);
        Serial.print(", ");
        Serial.println(calib.accelBias[2]);
        Serial.println("Gyro biases X/Y/Z: ");
        Serial.print(calib.gyroBias[0]);
        Serial.print(", ");
        Serial.print(calib.gyroBias[1]);
        Serial.print(", ");
        Serial.println(calib.gyroBias[2]);
        delay(2000);
        IMU.init(calib, IMU_ADDR);

        // Get the dead zone reading the worst values while still for 1.5 seconds
        //AccelData tmp;
        //float t = millis();

        /*
        while (millis() - t <= 2000) {
            IMU.update();
            IMU.getAccel(&tmp);
            deadZone[0] = max(deadZone[0], abs(tmp.accelX) * 5);
            deadZone[1] = max(deadZone[1], abs(tmp.accelY) * 5);
            deadZone[2] = max(deadZone[2], abs((float)(tmp.accelZ - 1.0)) * 5);
            delay(100);
        }

        Serial.print("Dead zone: ");
        Serial.print(deadZone[0], 6);
        Serial.print(", ");
        Serial.print(deadZone[1], 6);
        Serial.print(", ");
        Serial.println(deadZone[2], 6);
        */
    }
    //filter.begin(B_madgwick);
}

/*
void getAttitude(quat_t *quat, attitude_t *att) {
    AccelData IMUAccel;
    GyroData IMUGyro;

    IMU.update();
    unsigned long currentTime = micros();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);

    // Save quaternions from madgwick filter
    quat->x = filter.getQuatX();
    quat->y = filter.getQuatY();
    quat->z = filter.getQuatZ();
    quat->w = filter.getQuatW();
    quat->dt = (currentTime >= quat->t) ? (currentTime - quat->t) / 1000000.0f : (currentTime + (ULONG_MAX - quat->t + 1)) / 1000000.0f;
    quat->t = currentTime;

    // Save euler angles from quaternion
    att->roll = atan2(2.0f * (quat->w * quat->x + quat->y * quat->z), 1.0f - 2.0f * (quat->x * quat->x + quat->y * quat->y)) * 57.29577951;
    att->pitch = asin(constrain(2.0f * (quat->w * quat->y - quat->z * quat->x), -0.999999, 0.999999)) * 57.29577951;
    att->yaw = atan2(2.0f * (quat->w * quat->z + quat->x * quat->y), 1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z)) * 57.29577951;
    att->t = currentTime;
}
*/

void getAcceleration(vec_t *accel) {
    IMU.update();
    unsigned long currentTime = micros();

    AccelData tmp;

    IMU.getAccel(&tmp);

    // Save data considering the dead zone
    accel->x = tmp.accelX;
    accel->y = tmp.accelY;
    accel->z = tmp.accelZ;
    accel->dt = (currentTime >= accel->t) ? (currentTime - accel->t) / 1000000.0f : (currentTime + (ULONG_MAX - accel->t + 1)) / 1000000.0f;
    accel->t = currentTime;
}

void getGyro(vec_t *gyro) {
    IMU.update();
    unsigned long currentTime = micros();

    GyroData tmp;

    IMU.getGyro(&tmp);

    gyro->x = tmp.gyroX;
    gyro->y = tmp.gyroY;
    gyro->z = tmp.gyroZ;
    gyro->dt = (currentTime >= gyro->t) ? (currentTime - gyro->t) / 1000000.0f : (currentTime + (ULONG_MAX - gyro->t + 1)) / 1000000.0f;
    gyro->t = currentTime;
}

// Debug functions
void printIMUData(vec_t *data, const char *unit) {
    Serial.print(data->x);
    Serial.print(unit);
    Serial.print(", ");
    Serial.print(data->y);
    Serial.print(unit);
    Serial.print(", ");
    Serial.print(data->z);
    Serial.print(unit);
    Serial.print(", Time:");
    Serial.print(data->dt);
    Serial.println("s");
}

void printIMUData(quat_t *quat) {
    Serial.print(quat->w);
    Serial.print(", ");
    Serial.print(quat->x);
    Serial.print(", ");
    Serial.print(quat->y);
    Serial.print(", ");
    Serial.println(quat->z);
}

void printIMUData(attitude_t *att) {
    Serial.print(att->roll);
    Serial.print(", ");
    Serial.print(att->pitch);
    Serial.print(", ");
    Serial.println(att->yaw);
}

void logIMU(vec_t *pos, vec_t *speed, vec_t *accel) {
    Serial.print(pos->x, 4);
    Serial.print(",");
    Serial.print(pos->y, 4);
    Serial.print(",");
    Serial.print(pos->z, 4);
    Serial.print(",");
    Serial.print(speed->x, 4);
    Serial.print(",");
    Serial.print(speed->y, 4);
    Serial.print(",");
    Serial.print(speed->z, 4);
    Serial.print(",");
    Serial.print(accel->x, 4);
    Serial.print(",");
    Serial.print(accel->y, 4);
    Serial.print(",");
    Serial.print(accel->z, 4);
    Serial.println();
}