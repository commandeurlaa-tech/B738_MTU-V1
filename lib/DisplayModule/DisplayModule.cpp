#include "Core.h"
#include "Commands.h"

void onDisplay() {
  int dataID = messenger.readInt32Arg();  // Data ID

  if (dataID == 22) {  // IAS OVERSPEED
    int dataVal = messenger.readInt32Arg();
    dataVal = messenger.readInt32Arg();
    IASoverspeed = messenger.readBoolArg();
    if (IASoverspeed == true) {
      ALTWARN = 1;
    } else {
      ALTWARN = 0;
    }

  } else if (dataID == 23) {  // IAS UNDERSPEED
    int dataVal = messenger.readInt32Arg();
    dataVal = messenger.readInt32Arg();
    IASunderspeed = messenger.readBoolArg();
    if (IASunderspeed == true) {
      ALTWARN = 2;
    } else {
      ALTWARN = 0;
    }
  } else if (dataID == 25) {  //   IAS BLANK
    int dataVal = messenger.readInt32Arg();
    dataVal = messenger.readInt32Arg();
    IAS_blank = messenger.readBoolArg();
    if (IAS_blank == true) {
      disp_IAS.clear();
    }
  }

  else if (dataID == 26) {
    int32_t dataVal = messenger.readInt16Arg();
    dataVal = messenger.readInt32Arg();
    dataVal = messenger.readInt32Arg();
    if (HDG_BANK_SEL == 1) {
      disp_HDG.showNumber(dataVal, true, 3, 0);
    } else {
      if (dataVal == 0) {
        disp_HDG.showString("A10");
      } else if (dataVal == 1) {
        disp_HDG.showString("A15");
      } else if (dataVal == 2) {
        disp_HDG.showString("A20");
      } else if (dataVal == 3) {
        disp_HDG.showString("A25");
      } else if (dataVal == 4) {
        disp_HDG.showString("A30");
      }
    }
  }

  else if (dataID == 27) {
    int32_t dataVal = messenger.readInt16Arg();
    dataVal = messenger.readInt32Arg();
    dataVal = messenger.readFloatArg();
    dataVal = dataVal + 0.5;
    if (IAS_blank == false) {
      if (dataVal < 1000) {

        disp_IAS.showNumber(dataVal, false);
        uint8_t dots = 0b00010000;
        disp_IAS.showNumberDec(dataVal, dots);
      } else {
        dataVal = dataVal / 100;
        disp_IAS.showNumber(dataVal, false);
      }
      if (ALTWARN == 1) {
        disp_IAS.showString("b", 2, 1);
      } else if (ALTWARN == 2) {
        disp_IAS.showString("a", 2, 1);
      } else {
        disp_IAS.showString(" ", 2, 1);
      }
    } else {
      disp_IAS.clear();
    }
  }

  else if (dataID == 28) {
    int32_t dataVal = messenger.readInt16Arg();
    dataVal = messenger.readInt32Arg();
    dataVal = messenger.readInt32Arg();
    if (dataVal >= 1) {
      disp_CRS_L.showNumber(dataVal, true, 3, 0);
    } else {
      disp_CRS_L.showString("000");
    }
  } else if (dataID == 29) {
    int32_t dataVal = messenger.readInt16Arg();
    dataVal = messenger.readInt32Arg();
    HDG_BANK_SEL = messenger.readInt32Arg();
  } else {
  }
}