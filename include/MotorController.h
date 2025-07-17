#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    int pulPin;
    int dirPin;
    int enPin;
    int pwmChannel;
    int pwmFreq;
    int pwmResolution;

    MotorController(int pulPin, int dirPin, int enPin, int pwmChannel,
                    int pwmFreq = 500, int pwmResolution = 8)
        : pulPin(pulPin), dirPin(dirPin), enPin(enPin), pwmChannel(pwmChannel),
          pwmFreq(pwmFreq), pwmResolution(pwmResolution) {}

    void init() {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enPin, OUTPUT);

        digitalWrite(enPin, HIGH); // start disabled

        ledcSetup(pwmChannel, pwmFreq, pwmResolution);
        ledcAttachPin(pulPin, pwmChannel);
    }

    void enable() {
        digitalWrite(enPin, LOW);
    }

    void disable() {
        digitalWrite(enPin, HIGH);
    }

    void setDirection(bool forward) {
        digitalWrite(dirPin, forward ? HIGH : LOW);
    }

    void setPulseFrequency(int frequency) {
      pwmFreq = frequency;
      ledcSetup(pwmChannel, pwmFreq, pwmResolution);
      ledcAttachPin(pulPin, pwmChannel);
    }

  void setSpeed(uint8_t duty) {
    if (duty > (1 << pwmResolution) - 1) duty = (1 << pwmResolution) - 1;
    ledcWrite(pwmChannel, duty);
  }
};

#endif // MOTORCONTROLLER_H