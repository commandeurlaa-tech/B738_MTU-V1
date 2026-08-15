#include "Core.h"
#include "Commands.h"
#include "ServoModule.h"

int Brakevalue = 0;
float LeftBrake = 0.0;
float RightBrake = 0.0;
float ParkingBrake = 0.0;
bool BothBrakesPreviouslyPressed = false;
bool BrakeActionDone = false;
bool ServoActive = false;
unsigned long ServoStartTime = 0;

void checkBrakeCondition()
{
    bool BothBrakesPressed =
        (LeftBrake >= 0.9 && RightBrake >= 0.9);

    // Alleen reageren op de overgang:
    // beide remmen waren los en zijn nu beide ingedrukt.
    if (BothBrakesPressed && !BothBrakesPreviouslyPressed)
    {
        if (ParkingBrake >= 0.9)
        {
        //    messenger.sendCmd(kDebug,
        //        "***** PARKING BRAKE RELEASE - SERVO *****");

            myServo.write(0);

            ServoActive = true;
            ServoStartTime = millis();
        }
        else
        {
      //      messenger.sendCmd(kDebug,
      //          "BOTH BRAKES PRESSED - PARKING BRAKE IS OFF");
        }
    }

    BothBrakesPreviouslyPressed = BothBrakesPressed;
}


void updateServo()
{
    if (ServoActive)
    {
        if (millis() - ServoStartTime >= 400)
        {
            myServo.write(90);

            ServoActive = false;

          //  messenger.sendCmd(kDebug,
           //     "***** SERVO TERUG NAAR 90 *****");
        }
    }
}

void onIncomingData()
{
/*// Voorbeeld: De PMDG simulator meldt dat de Autothrust (A/T) IS INGESCHAKELD
// We zetten de motoren voor Throttle Links (0) en Rechts (1) 'aan' (spoelen bekrachtigd)
setStepperEnable(0, true);
setStepperEnable(1, true);

// Voorbeeld: PMDG stuurt de nieuwe stand van de gashendels door
setStepperTarget(0, 650); // Zet linker hendel op positie 650
setStepperTarget(1, 620); // Zet rechter hendel op positie 620

// Voorbeeld: De piloot zet de Autothrust handmatig UIT (A/T Disengage)
// De motoren vallen direct stroomloos en zijn weer met de hand te bedienen
setStepperEnable(0, false);
setStepperEnable(1, false);
*/


  int dataID = messenger.readInt32Arg(); // Data ID
//  messenger.sendCmd(kDebug, ">>>>> dataID: " + String(Brakevalue));
  if (dataID == 20)
  {
    float fldataVal = messenger.readFloatArg(); // Led value
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
      //  disp_IAS.setBrightness(dimvalue);
      // disp_CRS_L.setBrightness(dimvalue);
      // disp_HDG.setBrightness(dimvalue);
      olddimvalue = dimvalue;
    }
  }

  if (dataID == 21)
  {
    int dimvalue = messenger.readInt32Arg(); // 0–255 (bijv)

    // PWM output
    analogWrite(4, dimvalue);

    // schaal naar display brightness
    int brightness = round(dimvalue / 40);

    if (olddimvalue != brightness)
    {
      olddimvalue = brightness;
    }

    digitalWrite(13, HIGH);
  }

  if (dataID == 22)
  {
    int Dimvalue = messenger.readInt32Arg();
    //  messenger.sendCmd(kDebug, "STATS VLIEGTUIG" );
    //  messenger.sendCmd(kDebug,  Dimvalue);
    if (Dimvalue > 0)
    {
      analogWrite(9, 255);
    }
  }
if (dataID == kBrakeLeft)
{
    LeftBrake = messenger.readFloatArg();

  //  messenger.sendCmd(kDebug,
   //     "LEFT BRAKE: " + String(LeftBrake, 3));

    checkBrakeCondition();
}

if (dataID == kBrakeRight)
{
    RightBrake = messenger.readFloatArg();

 //   messenger.sendCmd(kDebug,
   //     "RIGHT BRAKE: " + String(RightBrake, 3));

    checkBrakeCondition();
}

if (dataID == kParkingBrake)
{
    ParkingBrake = messenger.readFloatArg();

   // messenger.sendCmd(kDebug,
    //    "PARKING BRAKE: " + String(ParkingBrake, 3));

   
}
}
