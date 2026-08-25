#include <Arduino.h>

#include "Core.h"
#include "SpadModule.h"
#include "InputModule.h"
#include "DataModule.h"
#include "StepperModule.h"

void setup()
{
    initHardware();

    initSteppers();

    attachCommandCallbacks();

    // Alleen voor de eerste test
    setStepperEnable(TRIM_WHEEL, true);

}

void loop()
{
    messenger.feedinSerialData();

    CheckAllButtons();

    updateServo();

    updateSteppers();
}