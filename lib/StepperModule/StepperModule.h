#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>
#include <AccelStepper.h>

class StepperModule {
private:
    AccelStepper _stepper;
    int _enaPin;
    int _switchPin;
    long _targetPos;

public:
    // Constructor waarin je de specifieke pinnen meegeeft per motor
    StepperModule(int pulPin, int dirPin, int enaPin, int switchPin);
    
    void begin(float maxSpeed, float acceleration);
    void update(); // Deze methode roep je straks continu aan in de hoofdloop
    void setTarget(long position);
};

#endif
