#include "StepperMotor.h"

StepperMotor::StepperMotor(int pulPin, int dirPin, int enPin, int pwmChannel, uint8_t id)
    : controller_(pulPin, dirPin, enPin, pwmChannel), id_(id) { 

    Wire.begin();
    if (!encoder_.begin()) {
        Serial.println("[StepperMotor] AS5600 encoder not found. Halting.");
        while (1);  // Halt execution
    }

    Serial.println("[StepperMotor] AS5600 initialized.");
    last_raw_angle_ = encoder_.readAngle();
    last_angle_deg_ = last_raw_angle_ * 360.0 / 4096.0;

    controller_.init();
    controller_.enable();
}

void StepperMotor::setPositionCommand(float pos, float vel, float acc) {
    cmd_pos_ = pos;
    cmd_vel_ = vel;
    cmd_acc_ = acc;
    mode_ = POSITION; 
}

void StepperMotor::setSpeedCommand(float vel, float acc) {
    cmd_vel_ = vel;
    cmd_acc_ = acc;
    mode_ = SPEED;
}

// Update encoder feedback
void StepperMotor::updatePosition() {
    uint16_t raw = encoder_.readAngle(); // 0 - 4095
    float angle_deg = raw * 360.0 / 4096.0;
    pos_ = angle_deg;  // You can convert to radians or steps if preferred
}


void StepperMotor::updateVelocity(float dt) {
    if (dt <= 0.0f) return;

    uint16_t raw = encoder_.readAngle();
    float angle_deg = raw * 360.0 / 4096.0;

    // Handle wraparound (0–360°)
    float delta = angle_deg - last_angle_deg_;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    vel_ = delta / dt; // degrees/sec (or convert to steps/sec)

    last_angle_deg_ = angle_deg;
}

void StepperMotor::runMotor(float dt) {
    
    updatePosition();
    updateVelocity(dt);

    if (mode_ == IDLE) {
        controller_.setSpeed(0); // Stop motor
        return;
    } else if (mode_ == POSITION) {
        runPositionControl(dt);
    } else if (mode_ == SPEED) {
        runSpeedControl(dt);
    }
}

void StepperMotor::runPositionControl(float dt) {
    float error = cmd_pos_ - pos_;
    static float integral = 0, prev_error = 0;

    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // Raw PID velocity demand
    float desired_vel = kp * error + ki * integral + kd * derivative;

    // --- Apply Velocity Constraints ---
    if (desired_vel > cmd_vel_) desired_vel = cmd_vel_;
    if (desired_vel < -cmd_vel_) desired_vel = -cmd_vel_;

    // Apply acceleration limiting 
    static float last_output_vel = 0;
    float max_delta_vel = cmd_acc_ * dt;  // max allowed velocity change this cycle

    float vel_change = desired_vel - last_output_vel;
    if (vel_change > max_delta_vel) vel_change = max_delta_vel;
    if (vel_change < -max_delta_vel) vel_change = -max_delta_vel;

    float output_vel = last_output_vel + vel_change;
    last_output_vel = output_vel;

    // --- Send to MotorController ---
    controller_.setDirection(output_vel >= 0);
    controller_.setPulseFrequency(fabs(output_vel));  // Hz ~ steps/sec
    controller_.setSpeed(128); // 50% duty for a clean square wave

}

void StepperMotor::runSpeedControl(float dt) {
    
    float vel_error = cmd_vel_ - vel_; // current vel from encoder
    static float integral = 0, prev_error = 0;

    integral += vel_error * dt;
    float derivative = (vel_error - prev_error) / dt;
    prev_error = vel_error;

    float target_vel = kp * vel_error + ki * integral + kd * derivative;

    // Clamp acceleration
    static float last_output_vel = 0;
    float max_delta_vel = cmd_acc_ * dt;
    float vel_change = target_vel - last_output_vel;
    if (vel_change > max_delta_vel) vel_change = max_delta_vel;
    if (vel_change < -max_delta_vel) vel_change = -max_delta_vel;

    float output_vel = last_output_vel + vel_change;
    last_output_vel = output_vel;

    controller_.setDirection(output_vel >= 0);
    controller_.setPulseFrequency(fabs(output_vel));
    controller_.setSpeed(128);
}