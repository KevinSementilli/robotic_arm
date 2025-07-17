#include <Arduino.h>
#include "CANbus.h"

#define DEBUG 1
#if DEBUG
    #include "test.cpp"
#endif

void setup() {

    #if DEBUG
        includeTesting(); // Run unit tests
    #endif

}

void loop() {
    
}