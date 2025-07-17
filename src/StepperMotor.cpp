#include "StepperMotor.h"

StepperMotor::StepperMotor(int pulPin, int dirPin, int enPin, int pwmChannel, uint8_t id)
    : controller_(pulPin, dirPin, enPin, pwmChannel), id_(id),
      encoder_(id == 0 ? &Wire : &Wire1) {

    // --- Select I2C pins based on motor ID ---
    if (id_ == 0) {
        Wire.begin(21, 22); // SDA=21, SCL=22 for Motor 0
    } else if (id_ == 1) {
        Wire1.begin(25, 26); // SDA=25, SCL=26 for Motor 1 (use Wire1)
    } else {
        Serial.printf("[StepperMotor] Invalid ID %d\n", id_);
        while (1);
    }

    // --- Encoder comms setup ---
    if (!encoder_.begin()) {
        Serial.printf("[StepperMotor %d] Encoder not found!\n", id_);
        while (1);
    } else {
        Serial.printf("[StepperMotor %d] Encoder initialized successfully\n", id_);
    }

    // --- Motor controller setup ---
    controller_.init();
    controller_.enable();

    // --- Initialize encoder position tracking ---
    last_raw_angle_ = encoder_.readAngle();
    last_angle_deg_ = last_raw_angle_ * 360.0f / 4096.0f;
}

void StepperMotor::calibrate() {

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
    static float last_angle = 0;
    static int32_t turn_count = 0;

    uint16_t raw = encoder_.readAngle();   // 0–4095
    float angle_deg = raw * 360.0 / 4096.0;

    // --- Detect wrap-around ---
    float delta = angle_deg - last_angle;

    if (delta > 180.0f) {
        turn_count -= 1; // CCW rotation decreases turn count
    } else if (delta < -180.0f) {
        turn_count += 1; // CW rotation increases turn count
    }

    last_angle = angle_deg;

    // --- Continuous absolute position (degrees) ---
    pos_ = turn_count * 360.0f + angle_deg;
}

void StepperMotor::updateVelocity(float dt) {
    if (dt <= 0.0f) return;

    uint16_t raw = encoder_.readAngle();
    float angle_deg = raw * 360.0f / 4096.0f;

    // --- Track cumulative position (multi-turn) ---
    float delta = angle_deg - last_angle_deg_;
    if (delta > 180.0f) {
        delta -= 360.0f;  // CW rollover
    } else if (delta < -180.0f) {
        delta += 360.0f;  // CCW rollover
    }

    cumulative_angle_deg_ += delta;

    // --- Velocity calculation ---
    vel_ = delta / dt;  // degrees per second (instantaneous velocity)

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
    // --- PID Outer Loop ---
    float error = cmd_pos_ - pos_;
    static float integral = 0, prev_error = 0;

    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float desired_vel = kp_ * error + ki_ * integral + kd_ * derivative;

    // Limit max velocity (target)
    if (desired_vel > cmd_vel_) desired_vel = cmd_vel_;
    if (desired_vel < -cmd_vel_) desired_vel = -cmd_vel_;

    // --- Apply Acceleration Limiting ---
    static float last_output_vel = 0;
    float max_delta_vel = computeAccel(dt);  

    float vel_change = desired_vel - last_output_vel;
    if (vel_change > max_delta_vel) vel_change = max_delta_vel;
    if (vel_change < -max_delta_vel) vel_change = -max_delta_vel;

    float output_vel = last_output_vel + vel_change;
    last_output_vel = output_vel;

    // --- Send to Motor Controller ---
    controller_.setDirection(output_vel >= 0);
    controller_.setPulseFrequency(fabs(output_vel));
    controller_.setSpeed(128);
}

void StepperMotor::runSpeedControl(float dt) {
    // --- PID Velocity Loop ---
    float vel_error = cmd_vel_ - vel_;
    static float integral = 0, prev_error = 0;

    integral += vel_error * dt;
    float derivative = (vel_error - prev_error) / dt;
    prev_error = vel_error;

    float target_vel = kp_ * vel_error + ki_ * integral + kd_ * derivative;

    // --- Apply Datasheet Acceleration Limiting ---
    static float last_output_vel = 0;
    float max_delta_vel = computeAccel(dt);

    float vel_change = target_vel - last_output_vel;
    if (vel_change > max_delta_vel) vel_change = max_delta_vel;
    if (vel_change < -max_delta_vel) vel_change = -max_delta_vel;

    float output_vel = last_output_vel + vel_change;
    last_output_vel = output_vel;

    controller_.setDirection(output_vel >= 0);
    controller_.setPulseFrequency(fabs(output_vel));
    controller_.setSpeed(128);
}

float StepperMotor::computeAccel(float dt) {
    if (cmd_acc_ <= 0) {
        return fabs(cmd_vel_);  // No ramping, instant jump
    }

    // Convert acc (0–255) to increments/sec
    float increment_interval_us = (256.0f - cmd_acc_) * 50.0f;  // μs
    float increments_per_sec = 1e6f / increment_interval_us;    // increments per second

    return increments_per_sec * dt;  // RPM increments allowed this cycle
}