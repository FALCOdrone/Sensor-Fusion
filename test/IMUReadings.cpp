#include "IMUReadings.h"

IMU::IMU(vec_t *gyro, quat_t *quat, attitude_t *att, vec_t *accel) {
    this->gyro = gyro;
    this->quat = quat;
    this->att = att;
    this->accel = accel;
}

void IMU::initialize() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("Finished IMU Initialization"));
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

quat_t IMU::getQuaternion() {
    Quaternion q;  // [w, x, y, z]         quaternion container
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    quat->w = q.w;
    quat->x = q.x;
    quat->y = q.y;
    quat->z = q.z;
    quat->dt = (currentTime >= quat->t) ? (currentTime - quat->t) / 1000.0f : (currentTime + (ULONG_MAX - quat->t + 1)) / 1000.0f;
    quat->t = currentTime;

    return *quat;
}

attitude_t IMU::getAttitude() {
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorFloat gravity;  // [x, y, z]            gravity vector
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    att->yaw = ypr[0];
    att->pitch = ypr[1];
    att->roll = ypr[2];
    att->dt = (currentTime >= att->t) ? (currentTime - att->t) / 1000.0f : (currentTime + (ULONG_MAX - att->t + 1)) / 1000.0f;
    att->t = currentTime;

    return *att;
}

vec_t *IMU::getRawAccel() {
    VectorInt16 aa;  // [x, y, z]            accel sensor measurements
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    mpu.dmpGetAccel(&aa, fifoBuffer);

    accel->x = aa.x / 16384.0f * 9.81f;
    accel->y = aa.y / 16384.0f * 9.81f;
    accel->z = aa.z / 16384.0f * 9.81f;
    accel->dt = (currentTime >= accel->t) ? (currentTime - accel->t) / 1000.0f : (currentTime + (ULONG_MAX - accel->t + 1)) / 1000.0f;
    accel->t = currentTime;

    return accel;
}

vec_t *IMU::getRawGyro() {
    VectorInt16 gy;  // [x, y, z]            gyro sensor measurements
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    mpu.dmpGetGyro(&gy, fifoBuffer);
    gyro->x = gy.x / 131.0f * PI / 180.0f;
    gyro->y = gy.y / 131.0f * PI / 180.0f;
    gyro->z = gy.z / 131.0f * PI / 180.0f;
    gyro->dt = (currentTime >= gyro->t) ? (currentTime - gyro->t) / 1000.0f : (currentTime + (ULONG_MAX - gyro->t + 1)) / 1000.0f;
    gyro->t = currentTime;

    return gyro;
}

vec_t *IMU::getRealAccel() {
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorInt16 aa;       // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;  // [x, y, z]            gravity vector
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    accel->x = aaReal.x / 16384.0f * 9.81f;
    accel->y = aaReal.y / 16384.0f * 9.81f;
    accel->z = aaReal.z / 16384.0f * 9.81f;
    accel->dt = (currentTime >= accel->t) ? (currentTime - accel->t) / 1000.0f : (currentTime + (ULONG_MAX - accel->t + 1)) / 1000.0f;
    accel->t = currentTime;

    return accel;
}

vec_t *IMU::getWorldAccel() {
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorInt16 aa;       // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;  // [x, y, z]            gravity vector
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    unsigned long currentTime = micros();
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    accel->x = aaWorld.x / 16384.0f * 9.81f;
    accel->y = aaWorld.y / 16384.0f * 9.81f;
    accel->z = aaWorld.z / 16384.0f * 9.81f;
    accel->dt = (currentTime >= accel->t) ? (currentTime - accel->t) / 1000.0f : (currentTime + (ULONG_MAX - accel->t + 1)) / 1000.0f;
    accel->t = currentTime;

    return accel;
}

vec_t IMU::getAcceleration() {

    vec_t *accelleration = getRealAccel();
    return *accelleration;
    
}

vec_t IMU::getGyro() {

    vec_t *angular_vel = getRawGyro();
    return *angular_vel;

}

// void printIMUData(vec_t *data, const char *unit);
// void printIMUData(vec_t *data);
// void printIMUData(bar_t *data);
// void printIMUData(quat_t *quat);
// void printIMUData(attitude_t *att);

// void logIMU(vec_t *pos, vec_t *speed, vec_t *accel);
