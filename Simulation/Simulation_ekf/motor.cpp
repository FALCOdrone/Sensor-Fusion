#include "motor.h"

// TODO: update with real transfer function between thrust and PWM
void initializeMotors() {
    pinMode(ESC1, OUTPUT);
    pinMode(ESC2, OUTPUT);
    pinMode(ESC3, OUTPUT);
    pinMode(ESC4, OUTPUT);
}

void driveMotors(float thrust[]) {
    // Motor 1
    analogWrite(ESC1, map(thrust[0], 0, 1, 0, 255));
    // Motor 2
    analogWrite(ESC2, map(thrust[1], 0, 1, 0, 255));
    // Motor 3
    analogWrite(ESC3, map(thrust[2], 0, 1, 0, 255));
    // Motor 4
    analogWrite(ESC4, map(thrust[3], 0, 1, 0, 255));
}

void throttleCut(unsigned long cutOffCmd, float pwm[], bool *armedFly) {
    // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
    /*
        Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
        minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
        called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
        the motors to anything other than minimum value. Safety first.

        channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
        channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
    */
    if ((cutOffCmd > 1500) || (*armedFly == false)) {
        *armedFly = false;
        for (int i = 0; i < 4; i++) {
            pwm[i] = 0.0;
        }
    }
}