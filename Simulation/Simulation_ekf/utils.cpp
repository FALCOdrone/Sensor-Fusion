#include "utils.h"

void setupBlink(int numBlinks, int upTime, int downTime) {
    // DESCRIPTION: Simple function to make LED on board blink as desired
    for (int j = 1; j <= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

void lpFilter(vec_t *current, vec_t *old, float b) {
    // DESCRIPTION: Simple low pass filter
    // INPUTS:
    //  current: current value
    //  old: old value
    //  b: filter coefficient
    // OUTPUTS:
    //  current: filtered value
    current->x = b * current->x + (1 - b) * old->x;
    current->y = b * current->y + (1 - b) * old->y;
    current->z = b * current->z + (1 - b) * old->z;
}

void lpFilter(attitude_t *current, attitude_t *old, float b) {
    current->roll = b * current->roll + (1 - b) * old->roll;
    current->pitch = b * current->pitch + (1 - b) * old->pitch;
    current->yaw = b * current->yaw + (1 - b) * old->yaw;
}

void lpFilter(quat_t *current, quat_t *old, float b) {
    current->w = b * current->w + (1 - b) * old->w;
    current->x = b * current->x + (1 - b) * old->x;
    current->y = b * current->y + (1 - b) * old->y;
    current->z = b * current->z + (1 - b) * old->z;
}

// Debug functions
void printData(vec_t *data, const char *unit) {
    Serial.print("X: ");
    Serial.print(data->x, 3);
    Serial.print(unit);
    Serial.print("\tY: ");
    Serial.print(data->y, 3);
    Serial.print(unit);
    Serial.print("\tZ: ");
    Serial.print(data->z, 3);
    Serial.print(unit);
    Serial.print("\tdt:");
    Serial.print(data->dt);
    Serial.println("ms");
}

void printData(quat_t *quat) {
    Serial.print("W: ");
    Serial.print(quat->w, 3);
    Serial.print("\tX: ");
    Serial.print(quat->x, 3);
    Serial.print("\tY: ");
    Serial.print(quat->y, 3);
    Serial.print("\tZ: ");
    Serial.print(quat->z, 3);
    Serial.print("\tdt: ");
    Serial.print(quat->dt);
    Serial.println("ms");
}

void printData(attitude_t *att) {
    Serial.print("R: ");
    Serial.print(att->roll, 3);
    Serial.print("\tP: ");
    Serial.print(att->pitch, 3);
    Serial.print("\tY: ");
    Serial.println(att->yaw, 3);
}

void printData(bar_t *bar) {
    Serial.print("Alt: ");
    Serial.print(bar->altitude, 3);
    Serial.print("\tPres: ");
    Serial.print(bar->pressure, 3);
    Serial.print("\tTemp: ");
    Serial.print(bar->temperature, 3);
    Serial.print("\tdt: ");
    Serial.print(bar->dt, 3);
    Serial.println("ms");
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

/*
void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX);
    Serial.print(F(" MagY:"));
    Serial.print(MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command:"));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command:"));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command:"));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command:"));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command:"));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command:"));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command:"));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}*/