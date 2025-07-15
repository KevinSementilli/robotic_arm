#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>
#include <CAN.h>

struct MotorCommand {
    float position;
    float velocity;
    float acceleration;
    bool valid = false;
};

class CANbus {
public:
    static inline MotorCommand motorCommands[2] = {};
    
    static bool begin(uint32_t baudRate = 500E3, int rxPin = 4, int txPin = 5) {
        CAN.setPins(rxPin, txPin);
        return CAN.begin(baudRate);
    }

    static void update() {
        if (CAN.parsePacket()) {
            uint32_t id = CAN.packetId();
            uint8_t data[12]; // Expect 3 floats
            int len = CAN.readBytes(data, CAN.packetDlc());
            processFrame(id, data, len);
        }
    }

    static MotorCommand getCommand(uint8_t motorID) {
        if (motorID > 1) return MotorCommand{};
        return motorCommands[motorID];
    }

    // Send feedback (position, velocity)
    static void sendFeedback(uint8_t motorID, float position, float velocity) {
        if (motorID > 1) return;

        uint8_t data[8];
        memcpy(&data[0], &position, sizeof(float));
        memcpy(&data[4], &velocity, sizeof(float));

        // Use a high CAN ID offset (e.g., 100 + motorID) to differentiate from incoming commands
        uint32_t feedbackID = 100 + motorID;
        CAN.beginPacket(feedbackID);
        CAN.write(data, sizeof(data));
        CAN.endPacket();
    }
 
private:
    static void processFrame(uint32_t id, const uint8_t* data, int len) {
        if (len < 12 || id > 1) return;

        MotorCommand cmd;
        memcpy(&cmd.position, &data[0], sizeof(float));
        memcpy(&cmd.velocity, &data[4], sizeof(float));
        memcpy(&cmd.acceleration, &data[8], sizeof(float));
        cmd.valid = true;

        motorCommands[id] = cmd;
    }
};

#endif // CANBUS_H
