#include "Core.h"
#include "Commands.h"
#include "LedModule.h"
#include "DataModule.h"
#include "SpadModule.h"

void onSpadEvent()
{
  char *szEvent = messenger.readStringArg();
  if (strcmp(szEvent, "START") == 0)
  { // SPAD tells device, it's ok to send input now
    isStarted = 1;
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("REFRESHDATA");
    messenger.sendCmdEnd();
    return;
  }
  if (strcmp(szEvent, "END") == 0)
  { // SPAD tells device it will exit now
    isStarted = 0;
    return;
  }
}

void onUnknownCommand()
{
  messenger.sendCmd(3, "UNKNOWN COMMAND");
}

void attachCommandCallbacks()
{
  messenger.sendCmd(kDebug, "ATTACHING CALLBACKS!");
  messenger.attach(kRequest, onIdentifyRequest);
  messenger.attach(kEvent, onSpadEvent);
  messenger.attach(kDebug, onUnknownCommand);
  messenger.attach(kLed, onDeviceLed);
  messenger.attach(kData, onIncomingData);
}

// ------------------------ PROCESS FUNCTIONS--------------------

void onIdentifyRequest()
{
  char *szRequest = messenger.readStringArg();

  if (strcmp(szRequest, "INIT") == 0)
  {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("SPAD");
    messenger.sendCmdArg(F("{2656302a-8aa7-4c93-bda6-7a12d883953e}"));   // bd465a74-6211-46bf-8f69-e2b5ae5c81e8 laatste cijfere was 9
    messenger.sendCmdArg("B737_MTU");                                    // DEVICE DISPLAY NAME
    messenger.sendCmdArg(2);                                             // SPAD SERIAL VERSION, DON'T CHANGE
    messenger.sendCmdArg("1");                                           // DEVICE VERSION NUMBER
    messenger.sendCmdArg("AUTHOR=2656302a-8aa7-4c93-bda6-7a12d883953e"); // AUTHOR ID
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "SCANSTATE") == 0)
  {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("STATESCAN");
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "PING") == 0)
  {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("PONG");
    messenger.sendCmdArg(messenger.readInt32Arg());
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "CONFIG") == 0)
  {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("OPTION");
    messenger.sendCmdArg("ISGENERIC=1");
    messenger.sendCmdArg("PAGESUPPORT=1");
    messenger.sendCmdArg("NO_DISPLAY_CLEAR=1");
    messenger.sendCmdEnd();
    //******************************

    // Expose DATA Value

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kPaneldimmer1);
    messenger.sendCmdArg("LVAR:BL_MainCA");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);   // This is a "1" or Command:1 from Spad list
    messenger.sendCmdArg("SUBSCRIBE");  // Subcommand..ADD - SUBSCRIBE - UNSUBSCRIBE - EMULATE
    messenger.sendCmdArg(KPanLightDim); // CMDID value defined at the top as 10
    messenger.sendCmdArg("LOCAL:PANELLIGHTDENSITY");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kGameState);
    messenger.sendCmdArg("LOCAL:GAMESTATE");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kBrakeLeft);
    messenger.sendCmdArg("SIMCONNECT:BRAKE LEFT POSITION");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kBrakeRight);
    messenger.sendCmdArg("SIMCONNECT:BRAKE RIGHT POSITION");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);
messenger.sendCmdArg("SUBSCRIBE");
messenger.sendCmdArg(kBrakeLeft);
messenger.sendCmdArg("SIMCONNECT:BRAKE LEFT POSITION");
messenger.sendCmdEnd();

messenger.sendCmdStart(kCommand);
messenger.sendCmdArg("SUBSCRIBE");
messenger.sendCmdArg(kBrakeRight);
messenger.sendCmdArg("SIMCONNECT:BRAKE RIGHT POSITION");
messenger.sendCmdEnd();

messenger.sendCmdStart(kCommand);
messenger.sendCmdArg("SUBSCRIBE");
messenger.sendCmdArg(kParkingBrake);
messenger.sendCmdArg("SIMCONNECT:BRAKE PARKING POSITION");
messenger.sendCmdEnd();

messenger.sendCmdStart(kCommand);
messenger.sendCmdArg("SUBSCRIBE");
messenger.sendCmdArg(kTrimWheel);
messenger.sendCmdArg("SIMCONNECT:ELEVATOR TRIM POSITION");
messenger.sendCmdEnd();

messenger.sendCmdStart(kCommand);
messenger.sendCmdArg("SUBSCRIBE");
messenger.sendCmdArg(kTrimIndicator);
messenger.sendCmdArg("SIMCONNECT:ELEVATOR TRIM INDICATOR");
messenger.sendCmdEnd();

    //----- CREATE ENCODERS --------------------------------------------------------------

    //----- CREATE BUTTONS-----------------------------------------------------

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(10);               // This is the button ID
    messenger.sendCmdArg(F("10AT_DISARM_1")); // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(11);              // This is the button ID
    messenger.sendCmdArg(F("11TOGA_1"));     // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON")); // Type
    messenger.sendCmdArg(F(""));           // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(12);               // This is the button ID
    messenger.sendCmdArg(F("AT_DISARM_2")); // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(13);              // This is the button ID
    messenger.sendCmdArg(F("13TOGA_2"));     // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON")); // Type
    messenger.sendCmdArg(F(""));           // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(14);              // This is the button ID
    messenger.sendCmdArg(F("14TRIMSTOP1"));  // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON")); // Type
    messenger.sendCmdArg(F(""));           // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(15);              // This is the button ID
    messenger.sendCmdArg(F("15TRIMSTOP2"));  // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON")); // Type
    messenger.sendCmdArg(F(""));           // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(16);                  // This is the button ID
    messenger.sendCmdArg(F("STAB_MAIN_ELEC")); // SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));         // Type
    messenger.sendCmdArg(F(""));               // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(17);                 // This is the button ID
    messenger.sendCmdArg(F("17TRIMAUTOPILOT")); // SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));        // Type
    messenger.sendCmdArg(F(""));              // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(18);               // This is the button ID
    messenger.sendCmdArg(F("18HORN_CUTOUT")); // SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(19);               // This is the button ID
    messenger.sendCmdArg(F("19FUELCUTOFF2")); // SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));      // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(20);               // This is the button ID
    messenger.sendCmdArg(F("20FUELCUFOFF1")); // SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));      // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(21);                 // This is the button ID
    messenger.sendCmdArg(F("21PARKING_BRAKE")); // SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));        // Type
    messenger.sendCmdArg(F(""));              // Behaviour
    messenger.sendCmdEnd();

    //----- CREATE DISPLAY --------------------------------------------------------------------
    // 0,OUTPUT,1,dAltitude,DISPLAY,SPAD_DISPLAY,LENGTH=20,ROWS=4,WIDTH=350,HEIGHT=120

    //----- CREATE LED'S --------------------------------------------------------------------

    createAllLEDs();

    //_________________________________________________________________________________________

    messenger.sendCmd(0, "CONFIG");

    return;
  }
}
