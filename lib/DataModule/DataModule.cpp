#include "Core.h"
#include "Commands.h"
#include "ServoModule.h"

// ============================================================
// BRAKE / PARKING BRAKE VARIABLES
// ============================================================

float LeftBrake = 0.0;
float RightBrake = 0.0;
float ParkingBrake = 0.0;


// ============================================================
// BRAKE STATE
// ============================================================

bool BothBrakesPressed = false;
bool BothBrakesPreviouslyPressed = false;


// ============================================================
// PARKING BRAKE STATE
// ============================================================

bool ParkingBrakeActive = false;

// TRUE = parking brake is actief en we hebben de rempedalen
//        sinds het aantrekken van de parking brake losgelaten.
//        Een volgende brake press mag dan de servo activeren.
bool ParkingBrakeReleaseArmed = false;


// ============================================================
// SERVO
// ============================================================

bool ServoActive = false;
unsigned long ServoStartTime = 0;

const int SERVO_RELEASE_POSITION = 0;
const int SERVO_HOME_POSITION = 90;

const unsigned long SERVO_TIME = 400;


// ============================================================
// BRAKE THRESHOLDS
// ============================================================
//
// Hysteresis voorkomt dat kleine fluctuaties in SPAD.neXt
// worden gezien als opnieuw indrukken van de remmen.
//

const float BRAKE_PRESS_THRESHOLD = 0.90;
const float BRAKE_RELEASE_THRESHOLD = 0.70;


// ============================================================
// CHECK BRAKE CONDITION
// ============================================================

void checkBrakeCondition()
{
    // --------------------------------------------------------
    // BEPAAL HUIDIGE BRAKE STATUS
    // --------------------------------------------------------

    if (!BothBrakesPreviouslyPressed)
    {
        // Remmen waren los.
        // Nieuwe brake press pas accepteren boven 0.90.

        BothBrakesPressed =
            (LeftBrake >= BRAKE_PRESS_THRESHOLD &&
             RightBrake >= BRAKE_PRESS_THRESHOLD);
    }
    else
    {
        // Remmen waren al ingedrukt.
        //
        // Ze blijven ingedrukt totdat BEIDE remmen
        // onder 0.70 komen.

        BothBrakesPressed =
            !(LeftBrake < BRAKE_RELEASE_THRESHOLD &&
              RightBrake < BRAKE_RELEASE_THRESHOLD);
    }


    // ========================================================
    // BEIDE REMMEN ZIJN NU OPNIEUW INGEDRUKT
    // ========================================================

    if (BothBrakesPressed && !BothBrakesPreviouslyPressed)
    {
        // ----------------------------------------------------
        // Alleen servo activeren als:
        //
        // 1. Parking brake actief is
        // 2. De pedalen sinds het zetten van de parking brake
        //    eerst zijn losgelaten
        //
        // Dit voorkomt dat het aantrekken van de parking brake
        // zelf de servo opnieuw activeert.
        // ----------------------------------------------------

        if (ParkingBrakeActive &&
            ParkingBrakeReleaseArmed)
        {
            // -----------------------------------------------
            // PARKING BRAKE RELEASE
            // -----------------------------------------------

            myServo.write(SERVO_RELEASE_POSITION);

            ServoActive = true;
            ServoStartTime = millis();


            // -----------------------------------------------
            // BELANGRIJK:
            //
            // Eenmaal gebruikt, meteen disarmen.
            // -----------------------------------------------

            ParkingBrakeReleaseArmed = false;


            // Debug indien gewenst:
            /*
            messenger.sendCmd(
                kDebug,
                "***** PARKING BRAKE RELEASE - SERVO *****"
            );
            */
        }
    }


    // ========================================================
    // BEIDE REMMEN ZIJN LOSGELATEN
    // ========================================================

    if (!BothBrakesPressed &&
        BothBrakesPreviouslyPressed)
    {
        // ----------------------------------------------------
        // Als parking brake actief is, zijn we nu klaar voor
        // een toekomstige brake press om de parking brake
        // te releasen.
        //
        // Dit is het belangrijke verschil met de vorige code.
        // ----------------------------------------------------

        if (ParkingBrakeActive)
        {
            ParkingBrakeReleaseArmed = true;


            // Debug indien gewenst:
            /*
            messenger.sendCmd(
                kDebug,
                "BRAKES RELEASED - PARKING BRAKE RELEASE ARMED"
            );
            */
        }
    }


    // Oude brake status bewaren
    BothBrakesPreviouslyPressed = BothBrakesPressed;
}


// ============================================================
// SERVO UPDATE
// ============================================================

void updateServo()
{
    if (ServoActive)
    {
        if (millis() - ServoStartTime >= SERVO_TIME)
        {
            myServo.write(SERVO_HOME_POSITION);

            ServoActive = false;


            // Debug indien gewenst:
            /*
            messenger.sendCmd(
                kDebug,
                "***** SERVO TERUG NAAR 90 *****"
            );
            */
        }
    }
}


// ============================================================
// INCOMING DATA FROM SPAD.NEXT / MESSENGER
// ============================================================

void onIncomingData()
{
    int dataID = messenger.readInt32Arg();


    // ========================================================
    // DATA ID 20
    // ========================================================

    if (dataID == 20)
    {
        float fldataVal = messenger.readFloatArg();

        fldataVal = fldataVal * 255;

        int32_t dataVal = round(fldataVal);

        analogWrite(9, dataVal);

        if (dataVal >= 1)
        {
            digitalWrite(13, HIGH);
        }
        else
        {
            digitalWrite(13, LOW);
        }

        int dimvalue = round(dataVal / 40);

        if (olddimvalue != dimvalue)
        {
            olddimvalue = dimvalue;
        }
    }


    // ========================================================
    // DATA ID 21
    // ========================================================

    if (dataID == 21)
    {
        int dimvalue = messenger.readInt32Arg();

        analogWrite(4, dimvalue);

        int brightness = round(dimvalue / 40);

        if (olddimvalue != brightness)
        {
            olddimvalue = brightness;
        }

        digitalWrite(13, HIGH);
    }


    // ========================================================
    // DATA ID 22
    // ========================================================

    if (dataID == 22)
    {
        int Dimvalue = messenger.readInt32Arg();

        if (Dimvalue > 0)
        {
            analogWrite(9, 255);
        }
    }


    // ========================================================
    // LEFT BRAKE
    // ========================================================

    if (dataID == kBrakeLeft)
    {
        LeftBrake = messenger.readFloatArg();

        checkBrakeCondition();
    }


    // ========================================================
    // RIGHT BRAKE
    // ========================================================

    if (dataID == kBrakeRight)
    {
        RightBrake = messenger.readFloatArg();

        checkBrakeCondition();
    }


    // ========================================================
    // PARKING BRAKE
    // ========================================================

    if (dataID == kParkingBrake)
    {
        ParkingBrake = messenger.readFloatArg();

        ParkingBrakeActive =
            (ParkingBrake >= 0.90);


        // ----------------------------------------------------
        // PARKING BRAKE WORDT AANGETROKKEN
        // ----------------------------------------------------
        //
        // Als de parking brake actief wordt terwijl de
        // beide rempedalen al zijn ingedrukt, dan zijn we
        // bezig met de NORMALE PROCEDURE om de parking brake
        // te zetten.
        //
        // Dus NIET meteen "release armed" maken.
        // ----------------------------------------------------

        if (ParkingBrakeActive)
        {
            if (BothBrakesPressed)
            {
                // Parking brake is zojuist gezet terwijl
                // beide remmen ingedrukt zijn.

                ParkingBrakeReleaseArmed = false;
            }
        }
        else
        {
            // ------------------------------------------------
            // Parking brake is daadwerkelijk OFF.
            // Servo hoeft niet meer gewapend te zijn.
            // ------------------------------------------------

            ParkingBrakeReleaseArmed = false;
        }
    }
}