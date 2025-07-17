#include "CANbus.h"

StepperMotor* CANbus::motor[2] = {nullptr, nullptr};

bool CANbus::begin(uint32_t baudRate, int rxPin, int txPin) {
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
    uint8_t crc_calc = computeCRC(can_id, buffer, len - 1);
    if (crc_calc != crc_received) {
        Serial.printf("[CANbus] CRC mismatch (calc=0x%02X, recv=0x%02X)\n", crc_calc, crc_received);
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

uint8_t CANbus::computeCRC(uint32_t can_id, const uint8_t* buffer, int len) {
    uint16_t crc_calc = can_id;

    for (int i = 0; i < len; i++) {
        crc_calc += buffer[i];
    }
    return static_cast<uint8_t>(crc_calc & 0xFF);
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

        case 0xF6: { // Speed mode command
        
            float vel = 0.0f, acc = 0.0f;
            memcpy(&vel, &data[0], sizeof(float));   // bytes 0-3
            memcpy(&acc, &data[4], sizeof(float));   // bytes 4-7

            motor[id]->setSpeedCommand(vel, acc);
            Serial.printf("[Motor %d] Speed cmd: vel=%.2f, acc=%.2f\n", id, vel, acc);
            break;
        }

        case 0xF5: { // Absolute position mode command
        
            float pos = 0.0f, vel = 0.0f, acc = 0.0f;
            memcpy(&pos, &data[0], sizeof(float));   // bytes 0-3
            memcpy(&vel, &data[4], sizeof(float));   // bytes 4-7
            memcpy(&acc, &data[8], sizeof(float));   // bytes 8-11

            motor[id]->setPositionCommand(pos, vel, acc);
            Serial.printf("[Motor %d] Pos cmd: pos=%.2f, vel=%.2f, acc=%.2f\n", id, pos, vel, acc);
            break;
        }

        default:
            motor[id]->mode_ = IDLE; // Unknown command, set to IDLE
            Serial.printf("[CANbus] Unknown code: 0x%02X\n", code);
            break;
    }
}

void CANbus::sendPosition(uint32_t id) {
    // Convert pos_ (degrees) to int48_t cumulative value
    int64_t position_value = static_cast<int64_t>(motor[id]->pos_); 
    // Ensure int48_t (only lower 6 bytes are used)
    
    uint8_t data[8] = {0};
    data[0] = 0x31; // Code

    // Fill bytes 2–7 (little-endian)
    for (int i = 0; i < 6; i++) {
        data[i + 1] = (position_value >> (8 * i)) & 0xFF;
    }

    // Compute CRC (includes code + value)
    data[7] = computeCRC(id, data, 7);

    // Send CAN packet
    CAN.beginPacket(id);
    CAN.write(data, 8);
    CAN.endPacket();

    Serial.printf("[CANbus] Sent Position (ID:%d, Value:%lld)\n", id, position_value);
}

void CANbus::sendSpeed(uint32_t id) {
    // Convert vel_ to RPM (positive = CCW, negative = CW as per datasheet)
    int16_t rpm = static_cast<int16_t>(motor[id]->vel_);

    uint8_t data[4] = {0};
    data[0] = 0x32; // Code

    // Fill bytes 2–3 (little-endian)
    data[1] = rpm & 0xFF;
    data[2] = (rpm >> 8) & 0xFF;

    // CRC (includes code + speed)
    data[3] = computeCRC(id, data, 3);

    // Send CAN packet
    CAN.beginPacket(id);
    CAN.write(data, 4);
    CAN.endPacket();

    Serial.printf("[CANbus] Sent Speed (ID:%d, RPM:%d)\n", id, rpm);
}