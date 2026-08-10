#include "InputModule.h"
#include "core.h"


void CheckAllButtons() {
  for (int i = 10; i <= 25; i++) {
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
