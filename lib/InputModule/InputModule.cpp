#include "InputModule.h"
#include "core.h"


void CheckAllButtons() {
  for (int i = 20; i <= 32; i++) {
    buttonStates[i] = digitalRead(i);
    if (buttonStates[i] != lastButtonStates[i]) {
      //   messenger.sendCmd(kDebug, i);
      if (lastButtonStates[i] == 0) {
        lastButtonStates[i] = 1;
        messenger.sendCmdStart(8);  // Channel no for sending input state to SPAD NEXT
        messenger.sendCmdArg(i);    // Input ID
        messenger.sendCmdArg(0);    // Value for off state
        messenger.sendCmdEnd();
      } else {
        lastButtonStates[i] = 0;
        messenger.sendCmdStart(8);  // Channel no for sending input state to SPAD NEXT
        messenger.sendCmdArg(i);    // Input ID
        messenger.sendCmdArg(1);    // Value for on state
        messenger.sendCmdEnd();
      }
    }
  }

  //delay(50);
}

//**************************************************************************************

void CheckAllEncoders() {
  CRS_L.tick();
  int newPos1 = CRS_L.getPosition();
  if (newPos1 != position1) {

    //  messenger.sendCmd(kDebug,  position1);
    messenger.sendCmdStart(8);                          // Channel no for sending input state to SPAD NEXT
    messenger.sendCmdArg(7);                            // Input ID
    messenger.sendCmdArg((int)(CRS_L.getDirection()));  // Value for on state
    messenger.sendCmdEnd();
    position1 = newPos1;
  }

  IAS.tick();
  int newPos2 = IAS.getPosition();
  if (newPos2 != position2) {
    position2 = newPos2;
    //   messenger.sendCmd(kDebug,  position1);
    messenger.sendCmdStart(8);                        // Channel no for sending input state to SPAD NEXT
    messenger.sendCmdArg(8);                          // Input ID
    messenger.sendCmdArg((int)(IAS.getDirection()));  // Value for on state
    messenger.sendCmdEnd();
    position2 = newPos2;
  }

  HDG.tick();
  int newPos3 = HDG.getPosition();
  if (newPos3 != position3) {
    position2 = newPos2;
    //   messenger.sendCmd(kDebug,  position1);
    messenger.sendCmdStart(8);                        // Channel no for sending input state to SPAD NEXT
    messenger.sendCmdArg(9);                          // Input ID
    messenger.sendCmdArg((int)(HDG.getDirection()));  // Value for on state
    messenger.sendCmdEnd();
    position3 = newPos3;
  }
}