#include "Core.h"
#include "Commands.h"

void onIncomingData() {
 int dataID = messenger.readInt32Arg();  // Data ID

  if (dataID == 20) {
    float fldataVal = messenger.readFloatArg();  // Led value
    fldataVal = fldataVal * 255;
    int32_t dataVal = round(fldataVal);
    analogWrite(4, dataVal);
    if (dataVal >= 1) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
    int dimvalue = round(dataVal / 40);
    if (olddimvalue != dimvalue) {
      disp_IAS.setBrightness(dimvalue);
      disp_CRS_L.setBrightness(dimvalue);
      disp_HDG.setBrightness(dimvalue);
      olddimvalue = dimvalue;
    }
  }

  if (dataID == 21) {
 int dimvalue = messenger.readInt32Arg();  // 0–255 (bijv)

// PWM output
analogWrite(4, dimvalue);

// schaal naar display brightness
int brightness = round(dimvalue / 40);

if (olddimvalue != brightness) {
  disp_IAS.setBrightness(brightness);
  disp_CRS_L.setBrightness(brightness);
  disp_HDG.setBrightness(brightness);
  olddimvalue = brightness;
}

digitalWrite(13, HIGH);
  }

  if (dataID == 22) {
    int Dimvalue = messenger.readInt32Arg();
   //  messenger.sendCmd(kDebug, "STATS VLIEGTUIG" );
   //  messenger.sendCmd(kDebug,  Dimvalue);
    if (Dimvalue > 0 ) {
      analogWrite(5, 255);
      analogWrite(6, 255);
      analogWrite(7, 255);
      analogWrite(8, 255);
      analogWrite(9, 255);
      analogWrite(10, 255);
      analogWrite(11, 255);
      analogWrite(12, 255);
    }
  }

  
}
