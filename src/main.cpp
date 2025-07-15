#include <Arduino.h>
#include "CANbus.h"
#include "MotorController.h"
#include "StepperMotor.h"

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
    Serial.println("========== [TEST 1] CANbus Message Retrieval ==========");

    // parse and store the latest frame
    CANbus::update(); 

    MotorCommand cmd = CANbus::getCommand(0);  // Test for motor ID 0
    if (cmd.valid) {
        Serial.println("[✓] Command received!");
        Serial.printf("  • Position: %.2f\n", cmd.position);
        Serial.printf("  • Velocity: %.2f\n", cmd.velocity);
        Serial.printf("  • Acceleration: %.2f\n", cmd.acceleration);
    } else {
        Serial.println("[✗] No valid command received.");
    }
}


void test_CAN_enc_feedback() {
    Serial.println("========== [TEST 2] CANbus Feedback Transmission ==========");
    float dummy_pos = 123.45;
    float dummy_vel = 67.89;
    Serial.println("[i] Sending feedback...");

    CANbus::sendFeedback(0, dummy_pos, dummy_vel);
    Serial.printf("[✓] Sent pos=%.2f, vel=%.2f on ID 100\n", dummy_pos, dummy_vel);
}

void test_MotorController() {
    Serial.println("========== [TEST 3] MotorController PWM Output ==========");

    MotorController controller(PUL_PIN, DIR_PIN, EN_PIN, PWM_CH);
    controller.init();
    controller.enable();

    controller.setDirection(true);
    controller.setPulseFrequency(1000); // 1kHz pulse train
    controller.setSpeed(128); // 50% duty

    Serial.println("[✓] PWM set to 1kHz, duty 50%, direction: forward");
    delay(2000);

    controller.setDirection(false);
    Serial.println("[✓] Direction set to reverse");

    delay(2000);
    controller.disable();
    Serial.println("[✓] Controller disabled");
}

void test_StepperMotor_pid() {
    Serial.println("========== [TEST 4] StepperMotor PID + Encoder ==========");

    // Set a manual command (bypassing CANbus)
    float target_position = 100.0; // degrees
    float target_velocity = 0.0;
    float target_acceleration = 0.0;

    stepper.setCommand(target_position, target_velocity, target_acceleration);

    float dt = 0.01; // 10 ms
    for (int i = 0; i < 100; ++i) {
        stepper.runPID(dt); // update() handles encoder + PID internally

        Serial.printf("Step %02d | Pos: %.2f | Error: %.2f | Vel: %.2f\n",
                      i, stepper.pos_, target_position - stepper.pos_, stepper.vel_);
        delay(10);
    }

    Serial.println("[✓] PID loop with encoder feedback complete");
}

void test_system() {
    Serial.println("========== [TEST 5] System Initialization ==========");
    
    // Initialize CAN bus
    if (CANbus::begin()) {
        Serial.println("[✓] CAN bus initialized successfully");
    } else {
        Serial.println("[✗] Failed to initialize CAN bus");
    }

    if (CANbus::updateCommand()) {
        Serial.println("[✓] Command updated from CAN bus");
    } else {
        Serial.println("[✗] No command received from CAN bus");
    }

}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("===== BEGINNING UNIT TESTS =====");

    CANbus::begin();  // Optional for test mode
    test_CAN_receiver();
    delay(1000);

    test_CAN_enc_feedback();
    delay(1000);

    test_MotorController();
    delay(1000);

    test_StepperMotor_pid();
    delay(1000);

    test_system();
    delay(1000);

    Serial.println("===== ALL UNIT TESTS COMPLETE =====");
}

void loop() {
    
}