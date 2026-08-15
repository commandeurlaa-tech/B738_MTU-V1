#include "StepperModule.h"

StepperModule::StepperModule(int pulPin, int dirPin, int enaPin, int switchPin) 
    : _stepper(AccelStepper::DRIVER, pulPin, dirPin) {
    _enaPin = enaPin;
    _switchPin = switchPin;
}

void StepperModule::begin(float maxSpeed, float acceleration) {
    pinMode(_switchPin, INPUT_PULLUP);
    _stepper.setEnablePin(_enaPin);
    _stepper.setPinsInverted(false, false, true); // Of false, false, false afhankelijk van driver
    _stepper.disableOutputs();
    _stepper.setMaxSpeed(maxSpeed);
    _stepper.setAcceleration(acceleration);
}

void StepperModule::update() {
    bool switchActive = (digitalRead(_switchPin) == LOW);

    if (switchActive) {
        _stepper.enableOutputs();
        _stepper.moveTo(_targetPos);
        _stepper.run();
    } else {
        _stepper.disableOutputs();
        _stepper.setCurrentPosition(0);
        _stepper.moveTo(0);
        _targetPos = 0;
    }
}

void StepperModule::setTarget(long position) {
    _targetPos = position;
}
