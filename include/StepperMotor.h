#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include "CANbus.h"
#include "MotorController.h"

class StepperMotor {
public:
    float cmd_pos_ = 0;
    float cmd_vel_ = 0;
    float cmd_acc_ = 0;
    float pos_ = 0;
    float vel_ = 0;
    float acc_ = 0;

    StepperMotor(int pulPin, int dirPin, int enPin, int pwmChannel, uint8_t id);

    // Command inputs from CAN or planner
    void setPositionCommand(MotorCommand& cmd);
    void setVelocityCommand(MotorCommand& cmd);

    // Periodic update for PID control execution
    void runPID(float dt);

    // External state update (e.g., encoder)
    void updatePosition();
    void updateVelocity(float dt);

private:

    MotorController controller_;
    float kp = 1;   
    float ki = 0;   
    float kd = 0; 

    AS5600 encoder_;
    uint16_t last_raw_angle_ = 0;
    float last_angle_deg_ = 0;
    uint8_t id_ = 0;
};

#endif // STEPPERMOTOR_H
