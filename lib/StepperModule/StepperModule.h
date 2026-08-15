#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>

const int NUM_STEPPERS = 6;

// Functies voor main.cpp
void initSteppers();   // Start alle pinnen en motoren op
void updateSteppers(); // Moet continu in de loop() draaien

// Functies om de waarden vanuit de PMDG status / SpadModule bij te werken
void setStepperTarget(int index, long position);
void setStepperEnable(int index, bool enabled); // Schakel motor aan (vast) of uit (vrijloop)

#endif
