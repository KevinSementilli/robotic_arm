#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include "CANbus.h"
#include "MotorController.h"

enum ControlMode { IDLE, POSITION, SPEED };

class StepperMotor {
public:
    float cmd_pos_ = 0;
    float cmd_vel_ = 0;
    float cmd_acc_ = 0;
    float pos_ = 0;
    float vel_ = 0;
    float acc_ = 0;
    ControlMode mode_ = IDLE;

    StepperMotor(int pulPin, int dirPin, int enPin, int pwmChannel, uint8_t id);

    void calibrate();

    // Command inputs from CAN
    void setPositionCommand(float pos, float vel, float acc);
    void setSpeedCommand(float vel, float acc);
    void setGain(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    float getKp() { return kp_;}
    float getKi() { return ki_; }
    float getKd() { return kd_; }

    void runMotor(float dt);
    void runPositionControl(float dt);
    void runSpeedControl(float dt);

    // External state update (encoder)
    void updatePosition();
    void updateVelocity(float dt);

private:

    // RPM increment per cycle
    float computeAccel(float dt);

    MotorController controller_;
    float kp_ = 1, ki_ = 0, kd_ = 0; // PID gains

    AS5600 encoder_;
    uint16_t last_raw_angle_ = 0;
    float last_angle_deg_ = 0;
    float cumulative_angle_deg_ = 0.0f;
    uint8_t id_ = 0;


};

#endif // STEPPERMOTOR_H