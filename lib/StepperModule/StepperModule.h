#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>

const int NUM_STEPPERS = 6;

enum StepperIndex
{
    TRIM_NEEDLE_1 = 0,
    TRIM_NEEDLE_2 = 1,
    THROTTLE_2    = 2,
    THROTTLE_1    = 3,
    SPEED_BRAKE   = 4,
    TRIM_WHEEL    = 5
};

struct StepperConfig
{
    long maxPosition;
    float speed;
    float acceleration;
    bool gespiegeld;
};

void initSteppers();
void updateSteppers();
void updateTrimWheel(float trimValue);
void updateTrimIndicator(float indicatorValue);

void setStepperTarget(int index, long position);
void setStepperEnable(int index, bool enabled);

void updateThrottleServos(bool active);
void updateThrottle1(float value);
void updateThrottle2(float value);

long getStepperPosition(int index);
bool isStepperEnabled(int index);



#endif