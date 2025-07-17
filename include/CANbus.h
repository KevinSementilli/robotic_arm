#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>
#include <CAN.h>
#include "StepperMotor.h"

class StepperMotor; // Forward declaration

class CANbus {
public:
    static StepperMotor* motor[2];
    
    // Initialize CAN bus with default parameters
    static bool begin(uint32_t baudRate = 500E3, int rxPin = 4, int txPin = 5);

    // Update command from CAN bus
    static bool updateCommand();

    static void sendPosition(uint32_t id);
    static void sendSpeed(uint32_t id);
 
private:

    static uint8_t computeCRC(uint32_t can_id, const uint8_t* buffer, int len);
    static void processFrame(uint32_t id, uint8_t code, const uint8_t* data, uint8_t len);

};

#endif // CANBUS_H