#include "StepperModule.h"
#include <AccelStepper.h>

// 1. Vul hier jouw exacte PUL en DIR pinnummers in voor je 6 motoren
AccelStepper steppers[NUM_STEPPERS] = {
    AccelStepper(AccelStepper::DRIVER, 29, 30),   // Motor 0 (bijv. Throttle Links)
    AccelStepper(AccelStepper::DRIVER, 31, 32),   // Motor 1 (bijv. Throttle Rechts)
    AccelStepper(AccelStepper::DRIVER, 33, 34),   // Motor 2 (bijv. Speedbrake)
    AccelStepper(AccelStepper::DRIVER, 35, 36),  // Motor 3 (bijv. Trimwiel)
    AccelStepper(AccelStepper::DRIVER, 37, 38), // Motor 4 (bijv. Flaps)
    AccelStepper(AccelStepper::DRIVER, 39, 40)  // Motor 5 (bijv. Extra/Spare)
};

// 2. Vul hier jouw exacte ENA pinnummers in (moeten in dezelfde volgorde staan als hierboven!)
const int enaPins[NUM_STEPPERS] = {41, 42, 43, 44, 45, 46}; // Voorbeeld: ENA pinnen voor de 6 motoren

// 3. Arrays voor de status van de motoren
long targetPositions[NUM_STEPPERS] = {0, 0, 0, 0, 0, 0};
bool stepperEnabled[NUM_STEPPERS]  = {false, false, false, false, false, false}; // Standaard in vrijloop

void initSteppers() {
    for (int i = 0; i < NUM_STEPPERS; i++) {
        steppers[i].setEnablePin(enaPins[i]);
        steppers[i].setPinsInverted(false, false, true); // Common Anode logica (Aanpasbaar naar false, false, false)
        steppers[i].disableOutputs();                    // Begin volledig stroomloos (vrijloop)
        
        // Stel hier de snelheden in die passen bij jouw throttle quadrant
        steppers[i].setMaxSpeed(1000.0);
        steppers[i].setAcceleration(500.0);
    }
}

void updateSteppers() {
    for (int i = 0; i < NUM_STEPPERS; i++) {
        // Controleer of PMDG heeft aangegeven dat deze motor actief moet zijn
        if (stepperEnabled[i]) {
            steppers[i].enableOutputs();            // Zet stroom op de spoelen (motor houdt positie vast)
            steppers[i].moveTo(targetPositions[i]); // Stuur naar de gewenste positie
            steppers[i].run();                      // Bereken en zet de stap
        } else {
            steppers[i].disableOutputs();           // Haal de stroom eraf zodat je hem met de hand kunt draaien
            
            // Houd de interne administratie van AccelStepper synchroon met de fysieke stand.
            // Omdat de motor met de hand verplaatst kan zijn, resetten we de positie tijdelijk naar 0 
            // zodat hij niet onverwacht wegschiet zodra de A/T weer inschakelt.
            steppers[i].setCurrentPosition(0);
            steppers[i].moveTo(0);
            targetPositions[i] = 0;
        }
    }
}

// Wordt aangeroepen als PMDG een nieuwe positie doorgeeft
void setStepperTarget(int index, long position) {
    if (index >= 0 && index < NUM_STEPPERS) {
        targetPositions[index] = position;
    }
}

// Wordt aangeroepen op basis van de PMDG Cockpit status (bijv. Autothrust engaged = true)
void setStepperEnable(int index, bool enabled) {
    if (index >= 0 && index < NUM_STEPPERS) {
        stepperEnabled[index] = enabled;
    }
}
