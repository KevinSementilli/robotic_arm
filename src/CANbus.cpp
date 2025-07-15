#include "CANbus.h"

bool CANbus::begin(uint32_t baudRate = 500E3, int rxPin = 4, int txPin = 5) {
    CAN.setPins(rxPin, txPin);
    return CAN.begin(baudRate);
}

bool CANbus::updateCommand() {
    if (!CAN.parsePacket()) return false;

    uint32_t can_id = CAN.packetId();
    uint8_t dlc = CAN.packetDlc();
    
    if (can_id > 1 || motor[can_id] == nullptr) {
        Serial.println("[CANbus] Wrong CAN ID or motor not initialized");
        return false;
    }

    if (dlc < 3) return false; // Minimum: code + CRC

    uint8_t buffer[8];
    int len = CAN.readBytes(buffer, dlc);

    uint8_t code = buffer[0];                 // Function code
    uint8_t crc_received = buffer[len - 1];   // Last byte is CRC

    // Compute CRC = (ID + all bytes except CRC) & 0xFF
    if (checkCRC(can_id, buffer, len - 1, crc_received)) {
        Serial.printf("[CANbus] CRC mismatch");
        return false;
    }

    // Extract data (if any)
    uint8_t data_len = len - 2;  // excluding code & CRC
    uint8_t data[6] = {0};
    if (data_len > 0) {
        memcpy(data, &buffer[1], data_len);
    }

    processFrame(can_id, code, data, data_len);
    return true;
}

bool CANbus::checkCRC(uint32_t can_id, const uint8_t* buffer, int len, uint8_t crc) {
    uint16_t crc_calc = can_id;

    for (int i = 0; i < len; i++) {
        crc_calc += buffer[i];
    }
    crc_calc &= 0xFF;
    return crc_calc == crc;
}

void CANbus::processFrame(uint32_t id, uint8_t code, const uint8_t* data, uint8_t len) {

    switch (code) {
        case 0x80: // Calibration command
            Serial.printf("[Motor %d] Calibration command received\n", id);
            motor[id]->calibrate();
            return;

        case 0x31: // send absolute position (feedback)
            sendPosition(id);
            return;

        case 0x32: // send velocity rpm (feedback)
            sendSpeed(id);
            return;

        case 0xF6: // Speed mode command

            float vel = 0.0f, acc = 0.0f;
            memcpy(&vel, &data[0], sizeof(float));   // bytes 0-3
            memcpy(&acc, &data[4], sizeof(float));   // bytes 4-7

            motor[id]->setSpeedCommand(vel, acc);
            Serial.printf("[Motor %d] Speed cmd: vel=%.2f, acc=%.2f\n", id, vel, acc);
            break;

        case 0xF5: // Absolute position mode command

            float pos = 0.0f, vel = 0.0f, acc = 0.0f;
            memcpy(&pos, &data[0], sizeof(float));   // bytes 0-3
            memcpy(&vel, &data[4], sizeof(float));   // bytes 4-7
            memcpy(&acc, &data[8], sizeof(float));   // bytes 8-11

            motor[id]->setPositionCommand(pos, vel, acc);
            Serial.printf("[Motor %d] Pos cmd: pos=%.2f, vel=%.2f, acc=%.2f\n", id, pos, vel, acc);
            break;

        default:
            Serial.printf("[CANbus] Unknown code: 0x%02X\n", code);
            return;
    }
}

void CANbus::sendPosition(uint32_t id) {        // VERIFY ME !!!!!

    float pos = motor[id]->pos_;
    uint8_t data[8];
    memcpy(&data[0], &pos, sizeof(float));

    CAN.beginPacket(id);
    CAN.write(data, sizeof(float));
    CAN.endPacket();
}

void CANbus::sendSpeed(uint32_t id) {       // VERIFY ME !!!!!

    float vel = motor[id]->vel_;
    uint8_t data[8];
    memcpy(&data[0], &vel, sizeof(float));

    CAN.beginPacket(id);
    CAN.write(data, sizeof(float));
    CAN.endPacket();
}