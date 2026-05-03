#include "Core.h"
#include "Commands.h"
#include "LedModule.h"
#include "DisplayModule.h"
#include "DataModule.h"
#include "SpadModule.h"


void onSpadEvent() {
  char *szEvent = messenger.readStringArg();
  if (strcmp(szEvent, "START") == 0) {  // SPAD tells device, it's ok to send input now
    isStarted = 1;
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("REFRESHDATA");
    messenger.sendCmdEnd();
    return;
  }
  if (strcmp(szEvent, "END") == 0) {  // SPAD tells device it will exit now
    isStarted = 0;
    return;
  }
}

void onUnknownCommand() {
  messenger.sendCmd(3, "UNKNOWN COMMAND");
}

void attachCommandCallbacks() {
  messenger.sendCmd(kDebug, "ATTACHING CALLBACKS!");
  messenger.attach(kRequest, onIdentifyRequest);
  messenger.attach(kEvent, onSpadEvent);
  messenger.attach(kDebug, onUnknownCommand);
  messenger.attach(kLed, onDeviceLed);
  messenger.attach(kDisplay, onDisplay);
  messenger.attach(kData, onIncomingData);
}

// ------------------------ PROCESS FUNCTIONS--------------------



void onIdentifyRequest() {
  char *szRequest = messenger.readStringArg();

  if (strcmp(szRequest, "INIT") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("SPAD");
    messenger.sendCmdArg(F("{61d13f6c-27bd-4f67-9d34-0fbfbe636f21}"));  // bd465a74-6211-46bf-8f69-e2b5ae5c81e8 laatste cijfere was 9
    messenger.sendCmdArg("B737_MCP_L");  // DEVICE DISPLAY NAME
    messenger.sendCmdArg(2);               // SPAD SERIAL VERSION, DON'T CHANGE
    messenger.sendCmdArg("1");             // DEVICE VERSION NUMBER
    messenger.sendCmdArg("AUTHOR=61d13f6c-27bd-4f67-9d34-0fbfbe636f21"); // AUTHOR ID
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "SCANSTATE") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("STATESCAN");
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "PING") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("PONG");
    messenger.sendCmdArg(messenger.readInt32Arg());
    messenger.sendCmdEnd();
    return;
  }

  if (strcmp(szRequest, "CONFIG") == 0) {
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

    messenger.sendCmdStart(kCommand);    // This is a "1" or Command:1 from Spad list
    messenger.sendCmdArg("SUBSCRIBE");   // Subcommand..ADD - SUBSCRIBE - UNSUBSCRIBE - EMULATE
    messenger.sendCmdArg(KPanLightDim);  // CMDID value defined at the top as 10
    messenger.sendCmdArg("LOCAL:PANELLIGHTDENSITY");
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kGameState);
    messenger.sendCmdArg("LOCAL:GAMESTATE");
    messenger.sendCmdEnd();

    
    //----- CREATE ENCODERS --------------------------------------------------------------

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(7);                  // This is the button ID
    messenger.sendCmdArg(F("CRS_L"));         //SPAD GUI Display name
    messenger.sendCmdArg(F("ENCODER"));       // Type
    messenger.sendCmdArg(F("SPAD_ENCODER"));  // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(8);                  // This is the button ID
    messenger.sendCmdArg(F("IAS"));           //SPAD GUI Display name
    messenger.sendCmdArg(F("ENCODER"));       // Type
    messenger.sendCmdArg(F("SPAD_ENCODER"));  // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(9);                  // This is the button ID
    messenger.sendCmdArg(F("HDG"));           //SPAD GUI Display name
    messenger.sendCmdArg(F("ENCODER"));       // Type
    messenger.sendCmdArg(F("SPAD_ENCODER"));  // Behaviour
    messenger.sendCmdEnd();


    //----- CREATE BUTTONS-----------------------------------------------------

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(20);               // This is the button ID
    messenger.sendCmdArg(F("HDG_BANK"));    //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dHDGBANK);  //29;
    messenger.sendCmdArg(F("D_HDGBANK"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=1,ROWS=1 ,HEIGHT=30,WIDTH=15"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(21);               // This is the button ID
    messenger.sendCmdArg(F("N1"));          //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(22);               // This is the button ID
    messenger.sendCmdArg(F("SPD"));         //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(23);               // This is the button ID
    messenger.sendCmdArg(F("VNAV"));        //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(24);               // This is the button ID
    messenger.sendCmdArg(F("LVL_CHG"));     //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(25);               // This is the button ID
    messenger.sendCmdArg(F("HGD_SEL"));     //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(26);               // This is the button ID
    messenger.sendCmdArg(F("APP"));         //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(27);               // This is the button ID
    messenger.sendCmdArg(F("VOR_LOC"));     //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(28);               // This is the button ID
    messenger.sendCmdArg(F("LNAV"));        //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(29);               // This is the button ID
    messenger.sendCmdArg(F("C_O"));         //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(30);               // This is the button ID
    messenger.sendCmdArg(F("SPD_INTV"));    //SPAD GUI Display name
    messenger.sendCmdArg(F("PUSHBUTTON"));  // Type
    messenger.sendCmdArg(F(""));            // Behaviour
    messenger.sendCmdEnd();


    //----- CREATE SWITCHES --------------------------------------------------------------

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(31);           // This is the button ID
    messenger.sendCmdArg(F("AT_ARM"));  //SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));  // Type
    messenger.sendCmdArg(F(""));        // Behaviour
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("INPUT"));
    messenger.sendCmdArg(32);            // This is the button ID
    messenger.sendCmdArg(F("FD_L_ON"));  //SPAD GUI Display name
    messenger.sendCmdArg(F("SWITCH"));   // Type
    messenger.sendCmdArg(F(""));         // Behaviour
    messenger.sendCmdEnd();
    //----- CREATE DISPLAY --------------------------------------------------------------------
    // 0,OUTPUT,1,dAltitude,DISPLAY,SPAD_DISPLAY,LENGTH=20,ROWS=4,WIDTH=350,HEIGHT=120


    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dIASblank);  //25;
    messenger.sendCmdArg(F("D_IASblank"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=1,ROWS=1 ,HEIGHT=30,WIDTH=15"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dIASoverspeed);
    messenger.sendCmdArg(F("D_IASOVERSPEED"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=1,ROWS=1 ,HEIGHT=30,WIDTH=15"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dIASunderspeed);  //dIASblank);
    messenger.sendCmdArg(F("D_IASUNDERSPEED"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=1,ROWS=1 ,HEIGHT=30,WIDTH=15"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dCourseL);
    messenger.sendCmdArg(F("D_CRS_L"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=3,ROWS=1 ,HEIGHT=30,WIDTH=45"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dIASMach);
    messenger.sendCmdArg(F("D_IASMACH"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=6,ROWS=1 ,HEIGHT=30,WIDTH=90"));
    messenger.sendCmdEnd();

    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("OUTPUT"));
    messenger.sendCmdArg(dHeading);
    messenger.sendCmdArg(F("D_CP_HEADING"));
    messenger.sendCmdArg(F("DISPLAY"));  // ID 15
    messenger.sendCmdArg(F("SPAD_DISPLAY"));
    messenger.sendCmdArg(F("LENGTH=3,ROWS=1 ,HEIGHT=30,WIDTH=45"));
    messenger.sendCmdEnd();

    //----- CREATE LED'S --------------------------------------------------------------------

    createAllLEDs();

    //_________________________________________________________________________________________
    
    messenger.sendCmd(0, "CONFIG");

    return;
  }
}
