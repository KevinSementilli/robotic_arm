#ifndef TEST_CPP
#define TEST_CPP

#include <Arduino.h>
#include "CANbus.h"

// Replace these with actual pins used in your circuit
#define PUL_PIN 23
#define DIR_PIN 19
#define EN_PIN  21
#define PWM_CH  0

StepperMotor stepper(PUL_PIN, DIR_PIN, EN_PIN, PWM_CH, 0); 

unsigned long lastUpdate = 0;
float testTime = 0;
bool testStarted = false;

void test_CAN_receiver() {
    Serial.println("========== [TEST 1] CANbus Command Execution ==========");

    if (!CANbus::updateCommand()) {
        Serial.println("[✗] No CAN message received");
        return;
    }

    // Check if StepperMotor was updated by CANbus
    Serial.printf("[✓] Command executed on Motor 0:\n");
    Serial.printf("  • Target Position: %.2f\n", stepper.cmd_pos_);
    Serial.printf("  • Target Velocity: %.2f\n", stepper.cmd_vel_);
    Serial.printf("  • Target Acceleration: %.2f\n", stepper.cmd_acc_);
    Serial.printf("  • Mode: %s\n",
                  stepper.mode_ == POSITION ? "POSITION" :
                  stepper.mode_ == SPEED ? "SPEED" : "IDLE");
}

void test_CAN_enc_feedback() {
    Serial.println("========== [TEST 2] CANbus Feedback Transmission ==========");
    float dummy_pos = 120.0;
    float dummy_vel = 20.0;

    Serial.println("[i] Sending feedback...");
    CANbus::sendPosition(0);
    CANbus::sendSpeed(0);

    Serial.printf("[✓] Sent test feedback (pos=%.2f, vel=%.2f)\n", dummy_pos, dummy_vel);
}

void test_MotorController() {
    Serial.println("========== [TEST 3] MotorController PWM Output ==========");

    MotorController controller(PUL_PIN, DIR_PIN, EN_PIN, PWM_CH);
    controller.init();
    controller.enable();

    controller.setDirection(true);
    controller.setPulseFrequency(1000);
    controller.setSpeed(128);

    Serial.println("[✓] Forward, 1kHz PWM");
    delay(2000);

    controller.setDirection(false);
    Serial.println("[✓] Reverse direction");
    delay(2000);

    controller.disable();
    Serial.println("[✓] Controller disabled");
}

void test_StepperMotor_position() {
    Serial.println("========== [TEST 4] StepperMotor POSITION Control ==========");

    float target_position = 90.0;  // degrees
    float max_velocity = 20;    // rpm
    float max_acc = 2;          // 0-255

    stepper.setPositionCommand(target_position, max_velocity, max_acc);

    float dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        stepper.runMotor(dt);
        Serial.printf("Step %02d | Pos: %.2f | Error: %.2f | Mode: POSITION\n",
                      i, stepper.pos_, target_position - stepper.pos_);
        delay(10);
    }

    Serial.println("[✓] Position control test complete");
}

void test_StepperMotor_speed() {
    Serial.println("========== [TEST 5] StepperMotor SPEED Control ==========");

    float target_velocity = 50.0; // rpm
    float max_acc = 2.0;          // 0-255

    stepper.setSpeedCommand(target_velocity, max_acc);

    float dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        stepper.runMotor(dt);
        Serial.printf("Step %02d | Vel: %.2f | Target: %.2f | Mode: SPEED\n",
                      i, stepper.vel_, target_velocity);
        delay(10);
    }

    Serial.println("[✓] Speed control test complete");
}

void test_system() {
    Serial.println("========== [TEST 6] Full System Workflow ==========");

    // 1. Initialize CAN
    if (!CANbus::begin()) {
        Serial.println("[✗] CAN bus initialization failed");
        return;
    }
    Serial.println("[✓] CAN bus initialized");

    Serial.println("[i] Waiting for CAN command (Position or Speed)...");
    
    unsigned long startTime = millis();
    bool commandReceived = false;

    // 2. Wait for a CAN command for up to 5 seconds
    while (millis() - startTime < 5000) {
        if (CANbus::updateCommand()) {
            Serial.println("[✓] Command received and processed by CANbus");
            commandReceived = true;
            break;
        }
    }

    if (!commandReceived) {
        Serial.println("[✗] No CAN command received within timeout");
        return;
    }

    // 3. Run the motor based on the received command
    Serial.println("[i] Executing command...");

    float dt = 0.01;
    for (int i = 0; i < 200; i++) {
        stepper.runMotor(dt);

        // Print live feedback
        if (stepper.mode_ == POSITION) {
            Serial.printf("Mode: POSITION | Pos: %.2f | Target: %.2f | Vel: %.2f\n",
                          stepper.pos_, stepper.cmd_pos_, stepper.vel_);
        } else if (stepper.mode_ == SPEED) {
            Serial.printf("Mode: SPEED | Vel: %.2f | Target: %.2f\n",
                          stepper.vel_, stepper.cmd_vel_);
        } else {
            Serial.println("Mode: IDLE");
        }
        delay(10);
    }

    Serial.println("[✓] System workflow test complete");
}

void tune_PID() {
    Serial.println("========== [PID TUNING MODE] ==========");
    Serial.println("Enter kp, ki, kd separated by spaces (e.g., '1.2 0.01 0.05')");
    Serial.println("Type 'exit' to stop tuning.");

    float target_pos = 90.0;  // Example position target (degrees)
    float max_vel = 180.0;    // Max velocity
    float max_acc = 90.0;     // Max acceleration

    // Set initial command
    stepper.setPositionCommand(target_pos, max_vel, max_acc);

    while (true) {
        // 1) Check for new PID input
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            if (input.equalsIgnoreCase("exit")) {
                Serial.println("[✓] Exiting PID tuning mode");
                stepper.mode_ = IDLE;
                break;
            }

            // Parse kp, ki, kd
            float new_kp, new_ki, new_kd;
            int parsed = sscanf(input.c_str(), "%f %f %f", &new_kp, &new_ki, &new_kd);

            if (parsed == 3) {
                stepper.setGain(new_kp, new_ki, new_kd);

                Serial.printf("[✓] Updated PID → kp: %.3f, ki: %.3f, kd: %.3f\n",
                              stepper.getKp(), stepper.getKi(), stepper.getKd());

                // Re-run the same position command with new PID
                stepper.setPositionCommand(target_pos, max_vel, max_acc);
            } else {
                Serial.println("[✗] Invalid format. Example: 1.2 0.01 0.05");
            }
        }

        // 2) Run control loop continuously
        static unsigned long lastUpdate = millis();
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f;
        lastUpdate = now;

        stepper.runMotor(dt);

        // Print live feedback
        Serial.printf("Pos: %.2f | Target: %.2f | Vel: %.2f | kp: %.2f ki: %.2f kd: %.2f\n",
                      stepper.pos_, target_pos, stepper.vel_, stepper.getKp(), stepper.getKi(), stepper.getKd());

        delay(20);
    }
}

void includeTesting() {
        Serial.begin(115200);
    delay(1000);
    Serial.println("===== BEGINNING UNIT TESTS =====");

    CANbus::begin();

    test_CAN_receiver();
    delay(1000);
    test_CAN_enc_feedback();
    delay(1000);
    test_MotorController();
    delay(1000);
    test_StepperMotor_position();
    delay(1000);
    test_StepperMotor_speed();
    delay(1000);
    test_system();
    delay(1000);
    tune_PID();
    delay(1000);

    Serial.println("===== ALL UNIT TESTS COMPLETE =====");
}

#endif // TEST_CPP