#ifndef CTRL_H
#define CTRL_H

#include "types.h"
#include <Arduino.h>

class controller {
    private:

        // Controller parameters (take note of defaults before modifying!):
        float i_limit = 25.0;   // Integrator saturation level, mostly for safety (default 25.0)

        float Kp_roll_angle = 0.2;    // Roll P-gain - angle mode
        float Ki_roll_angle = 0.3;    // Roll I-gain - angle mode
        float Kd_roll_angle = 0.05;   // Roll D-gain - angle mode (has no effect on controlANGLE2)
        float B_loop_roll = 0.9;      // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
        float Kp_pitch_angle = 0.2;   // Pitch P-gain - angle mode
        float Ki_pitch_angle = 0.3;   // Pitch I-gain - angle mode
        float Kd_pitch_angle = 0.05;  // Pitch D-gain - angle mode (has no effect on controlANGLE2)
        float B_loop_pitch = 0.9;     // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

        float Kp_roll_rate = 0.15;     // Roll P-gain - rate mode
        float Ki_roll_rate = 0.2;      // Roll I-gain - rate mode
        float Kd_roll_rate = 0.0002;   // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
        float Kp_pitch_rate = 0.15;    // Pitch P-gain - rate mode
        float Ki_pitch_rate = 0.2;     // Pitch I-gain - rate mode
        float Kd_pitch_rate = 0.0002;  // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

        float Kp_yaw = 0.3;      // Yaw P-gain
        float Ki_yaw = 0.05;     // Yaw I-gain
        float Kd_yaw = 0.00015;  // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

    public:

        void controlMixer(float throttleDes, float mCmdScaled[], attitude_t attPID);
        void controlANGLE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, PID_t *PID);
        void controlANGLE2(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, attitude_t *attIMUprev, PID_t *PIDol, PID_t *PIDil);
        void controlRATE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, vec_t *prevGyFro, attitude_t attIMU, PID_t *PID);
};

#endif