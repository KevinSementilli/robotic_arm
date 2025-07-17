#include <Arduino.h>
#include "CANbus.h"
#include "test.cpp"

#define DEBUG 1

void setup() {

    #if DEBUG
        includeTesting();
    #endif

}

void loop() {
    
}