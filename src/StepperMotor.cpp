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

// Set full motion command
void StepperMotor::setPositionCommand(MotorCommand& cmd) {
    cmd_pos_ = cmd.position;
    cmd_vel_ = cmd.velocity;
    cmd_acc_ = cmd.acceleration;
}

// Set only velocity command (optional mode)
void StepperMotor::setVelocityCommand(MotorCommand& cmd) {
    cmd_vel_ = cmd.velocity;
    cmd_acc_ = cmd.acceleration;
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


// Execute PID control and apply to motor
void StepperMotor::runPID(float dt) {
    updatePosition();
    updateVelocity(dt);

    // --- Get CAN Command for This Motor ---
    MotorCommand cmd = CANbus::getCommand(id_);
    if (!cmd.valid) return;

    setPositionCommand(cmd);

    // --- PID Calculation (Position loop) ---
    float error = cmd_pos_ - pos_;
    static float integral = 0;
    static float prev_error = 0;

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



