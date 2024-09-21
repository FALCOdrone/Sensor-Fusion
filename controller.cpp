#include "controller.h"

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

void controlMixer(float throttleDes, float mCmdScaled[], attitude_t attPID) {
    // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
    /*
     * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
     * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
     * should have -roll_PID-> Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
     * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
     * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands()
     * in preparation to be sent to the motor ESCs and servos.
     *
     *Relevant variables:
     *thro_des - direct thottle control
     *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
     *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
     *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
     */

    // Quad mixing - EXAMPLE
    mCmdScaled[0] = throttleDes - attPID.pitch + attPID.roll + attPID.yaw;  // Front Left
    mCmdScaled[1] = throttleDes - attPID.pitch - attPID.roll - attPID.yaw;  // Front Right
    mCmdScaled[2] = throttleDes + attPID.pitch - attPID.roll + attPID.yaw;  // Back Right
    mCmdScaled[3] = throttleDes + attPID.pitch + attPID.roll - attPID.yaw;  // Back Left
}

void controlANGLE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, PID_t *PID) {
    // DESCRIPTION: Computes control commands based on state error (angle)
    /*
     * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in
     * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
     * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
     * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
     * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
     * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
     * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
     * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
     */

    float dt = attIMU.dt;

    // Roll
    PID->p.roll = desiredAtt.roll - attIMU.roll;
    PID->i.roll = PID->iPrev.roll + PID->p.roll * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.roll = 0;
    }
    PID->i.roll = constrain(PID->i.roll, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.roll = gyro.x;
    PID->out.roll = 0.01 * (Kp_roll_angle * PID->p.roll + Ki_roll_angle * PID->i.roll - Kd_roll_angle * PID->d.roll);  // Scaled by .01 to bring within -1 to 1 range

    // Pitch
    PID->p.pitch = desiredAtt.pitch - attIMU.pitch;
    PID->i.pitch = PID->iPrev.pitch + PID->p.pitch * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.pitch = 0;
    }
    PID->i.pitch = constrain(PID->i.pitch, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.pitch = gyro.y;
    PID->out.pitch = .01 * (Kp_pitch_angle * PID->p.pitch + Ki_pitch_angle * PID->i.pitch - Kd_pitch_angle * PID->d.pitch);  // Scaled by .01 to bring within -1 to 1 range

    // Yaw, stablize on rate from GyroZ
    PID->p.yaw = desiredAtt.yaw - gyro.z;
    PID->i.yaw = PID->iPrev.yaw + PID->p.yaw * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.yaw = 0;
    }
    PID->i.yaw = constrain(PID->i.yaw, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.yaw = (PID->p.yaw - PID->pPrev.yaw) / dt;
    PID->out.yaw = .01 * (Kp_yaw * PID->p.yaw + Ki_yaw * PID->i.yaw + Kd_yaw * PID->d.yaw);  // Scaled by .01 to bring within -1 to 1 range

    // Update roll variables
    PID->iPrev.roll = PID->i.roll;
    // Update pitch variables
    PID->iPrev.pitch = PID->i.pitch;
    // Update yaw variables
    PID->pPrev.yaw = PID->p.yaw;
    PID->iPrev.yaw = PID->i.yaw;
}

void controlANGLE2(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, attitude_t *attIMUprev, PID_t *PIDol, PID_t *PIDil) {
    // DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
    /*
     * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
     * See the documentation for tuning this controller.
     */
    float dt = attIMU.dt;

    // Outer loop - PID on angle
    // Roll
    PIDol->p.roll = desiredAtt.roll - attIMU.roll;
    PIDol->i.roll = PIDol->iPrev.roll + PIDol->p.roll * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PIDol->i.roll = 0;
    }
    PIDol->i.roll = constrain(PIDol->i.roll, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PIDol->d.roll = (attIMU.roll - attIMUprev->roll) / dt;
    PIDol->out.roll = Kp_roll_angle * PIDol->p.roll + Ki_roll_angle * PIDol->i.roll;  // - Kd_roll_angle*PIDol->d.roll;

    // Pitch
    PIDol->p.pitch = desiredAtt.pitch - attIMU.pitch;
    PIDol->i.pitch = PIDol->iPrev.pitch + PIDol->p.pitch * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PIDol->i.pitch = 0;
    }
    PIDol->i.pitch = constrain(PIDol->i.pitch, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PIDol->d.pitch = (attIMU.pitch - attIMUprev->pitch) / dt;
    PIDol->out.pitch = Kp_pitch_angle * PIDol->p.pitch + Ki_pitch_angle * PIDol->i.pitch;  // - Kd_pitch_angle*derivative_pitch;

    // Apply loop gain, constrain, and LP filter for artificial damping
    float Kl = 30.0;
    PIDol->out.roll = Kl * PIDol->out.roll;
    PIDol->out.pitch = Kl * PIDol->out.pitch;
    PIDol->out.roll = constrain(PIDol->out.roll, -240.0, 240.0);
    PIDol->out.pitch = constrain(PIDol->out.pitch, -240.0, 240.0);
    PIDol->out.roll = (1.0 - B_loop_roll) * PIDol->outPrev.roll + B_loop_roll * PIDol->out.roll;
    PIDol->out.pitch = (1.0 - B_loop_pitch) * PIDol->outPrev.pitch + B_loop_pitch * PIDol->out.pitch;

    // Inner loop - PID on rate
    // Roll
    PIDil->p.roll = PIDol->out.roll - gyro.x;
    PIDil->i.roll = PIDil->iPrev.roll + PIDil->p.roll * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PIDil->i.roll = 0;
    }
    PIDil->i.roll = constrain(PIDil->i.roll, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PIDil->d.roll = (PIDil->p.roll - PIDil->pPrev.roll) / dt;
    PIDil->out.roll = .01 * (Kp_roll_rate * PIDil->p.roll + Ki_roll_rate * PIDil->i.roll + Kd_roll_rate * PIDil->d.roll);  // Scaled by .01 to bring within -1 to 1 range

    // Pitch
    PIDil->p.pitch = PIDol->out.pitch - gyro.y;
    PIDil->i.pitch = PIDil->iPrev.pitch + PIDil->p.pitch * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PIDil->i.pitch = 0;
    }
    PIDil->i.pitch = constrain(PIDil->i.pitch, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PIDil->d.pitch = (PIDil->p.pitch - PIDil->pPrev.pitch) / dt;
    PIDil->out.pitch = .01 * (Kp_pitch_rate * PIDil->p.pitch + Ki_pitch_rate * PIDil->i.pitch + Kd_pitch_rate * PIDil->d.pitch);  // Scaled by .01 to bring within -1 to 1 range

    // Yaw
    PIDil->p.yaw = desiredAtt.yaw - gyro.z;
    PIDil->i.yaw = PIDil->iPrev.yaw + PIDil->p.yaw * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PIDil->i.yaw = 0;
    }
    PIDil->i.yaw = constrain(PIDil->i.yaw, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PIDil->d.yaw = (PIDil->p.yaw - PIDil->pPrev.yaw) / dt;
    PIDil->out.yaw = .01 * (Kp_yaw * PIDil->p.yaw + Ki_yaw * PIDil->i.yaw + Kd_yaw * PIDil->d.yaw);  // Scaled by .01 to bring within -1 to 1 range

    // Update roll variables
    PIDol->iPrev.roll = PIDol->i.roll;
    PIDil->iPrev.roll = PIDil->i.roll;
    PIDol->pPrev.roll = PIDol->p.roll;
    PIDil->pPrev.roll = PIDil->p.roll;
    PIDol->outPrev.roll = PIDol->out.roll;
    // Update pitch variables
    PIDol->iPrev.pitch = PIDol->i.pitch;
    PIDil->iPrev.pitch = PIDil->i.pitch;
    PIDol->pPrev.pitch = PIDol->p.pitch;
    PIDil->pPrev.pitch = PIDil->p.pitch;
    PIDol->outPrev.pitch = PIDol->out.pitch;
    // Update yaw variables
    PIDol->pPrev.yaw = PIDol->p.yaw;
    PIDil->pPrev.yaw = PIDil->p.yaw;
    PIDol->iPrev.yaw = PIDol->i.yaw;
    PIDil->iPrev.yaw = PIDil->i.yaw;
}

void controlRATE(unsigned long throttleCmd /*channel_1_pwm */, attitude_t desiredAtt, vec_t gyro, attitude_t attIMU, PID_t *PID) {
    // DESCRIPTION: Computes control commands based on state error (rate)
    /*
     * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
     */
    float dt = attIMU.dt;

    // Roll
    PID->p.roll = desiredAtt.roll - gyro.x;
    PID->i.roll = PID->iPrev.roll + PID->p.roll * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.roll = 0;
    }
    PID->i.roll = constrain(PID->i.roll, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.roll = (PID->p.roll - PID->pPrev.roll) / dt;
    PID->out.roll = 0.01 * (Kp_roll_rate * PID->p.roll + Ki_roll_rate * PID->i.roll - Kd_roll_rate * PID->d.roll);  // Scaled by .01 to bring within -1 to 1 range

    // Pitch
    PID->p.pitch = desiredAtt.pitch - gyro.y;
    PID->i.pitch = PID->iPrev.pitch + PID->p.pitch * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.pitch = 0;
    }
    PID->i.pitch = constrain(PID->i.pitch, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.pitch = (PID->p.pitch - PID->pPrev.pitch) / dt;
    PID->out.pitch = .01 * (Kp_pitch_rate * PID->p.pitch + Ki_pitch_rate * PID->i.pitch - Kd_pitch_rate * PID->d.pitch);  // Scaled by .01 to bring within -1 to 1 range

    // Yaw, stablize on rate from GyroZ
    PID->p.yaw = desiredAtt.yaw - gyro.z;
    PID->i.yaw = PID->iPrev.yaw + PID->p.yaw * dt;
    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.yaw = 0;
    }
    PID->i.yaw = constrain(PID->i.yaw, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.yaw = (PID->p.yaw - PID->pPrev.yaw) / dt;
    PID->out.yaw = .01 * (Kp_yaw * PID->p.yaw + Ki_yaw * PID->i.yaw + Kd_yaw * PID->d.yaw);  // Scaled by .01 to bring within -1 to 1 range

    // Update roll variables
    PID->pPrev.roll = PID->p.roll;
    PID->iPrev.roll = PID->i.roll;
    // Update pitch variables
    PID->pPrev.pitch = PID->p.pitch;
    PID->iPrev.pitch = PID->i.pitch;
    // Update yaw variables
    PID->pPrev.yaw = PID->p.yaw;
    PID->iPrev.yaw = PID->i.yaw;
}