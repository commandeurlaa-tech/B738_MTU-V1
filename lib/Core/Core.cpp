#include "Core.h"
#include "Commands.h"
#include "ServoModule.h"


CmdMessenger messenger(Serial);

String authkey = "AUTHOR=2656302a-8aa7-4c93-bda6-7a12d883953e";

bool isReady = false;
bool isStarted = false;

int olddimvalue = 0;
int GameState = 0;
int DimState = 0;

int lastButtonStates[26] = {};
int buttonStates[26];

void initHardware()
{
    Serial.begin(115200);

    initServo();

    // Pin 28 is the "ENABLE" pin for the parking brake light
    pinMode(28, OUTPUT); 

    // Geen algemene OUTPUT/HIGH initialisatie meer voor 29 t/m 47.
    // StepperModule beheert zijn eigen STEP/DIR/ENA-pinnen.

    for (int i = 10; i <= 25; i++)
    {
        pinMode(i, INPUT_PULLUP);
        lastButtonStates[i] = 0;
    }

    delay(100);
}