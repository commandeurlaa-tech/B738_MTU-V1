#include <Arduino.h>
#include "Core.h"
#include "SpadModule.h"
#include "InputModule.h"
#include "DataModule.h"

void setup()
{
    initHardware();
    attachCommandCallbacks();
}

void loop()
{
    messenger.feedinSerialData();
    CheckAllButtons();  // <-- DEZE ONTBRAK
       updateServo();
}