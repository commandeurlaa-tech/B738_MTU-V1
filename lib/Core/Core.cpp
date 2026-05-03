#include "Core.h"
#include "Commands.h"
// #include <TM1637Display.h>

CmdMessenger messenger(Serial);

String authkey = "AUTHOR=61d13f6c-27bd-4f67-9d34-0fbfbe636f21";

bool isReady = false;
bool isStarted = false;
bool IAS_blank = false;
bool IASoverspeed = false;
bool IASunderspeed = false;
bool VSblank = true;
int ALTWARN = 0;
int HDG_BANK_SEL = 1;
char Angel;
int HDG_VAL;

int olddimvalue = 0;
int GameState = 0;
int DimState = 0;
int lastButtonStates[33] = {};
int buttonStates[33];

TM1637TinyDisplay6 disp_IAS(42, 43);
TM1637TinyDisplay disp_CRS_L(40, 41);
TM1637TinyDisplay disp_HDG(44, 45);

RotaryEncoder CRS_L(14, 15, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoder IAS(16, 17, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoder HDG(18, 19, RotaryEncoder::LatchMode::FOUR3);

long position1 = -999;
long position2 = -999;
long position3 = -999;

void initHardware()
{

  Serial.begin(115200);

  disp_IAS.setBrightness(2);
  disp_CRS_L.setBrightness(2);
  disp_HDG.setBrightness(2);
  disp_IAS.clear();
  disp_CRS_L.clear();
  disp_HDG.clear();

  // long position1 = -999;
  // long position2 = -999;
  // long position3 = -999;

  for (int current_pin = 4; current_pin <= 12; current_pin++) // Get all the pins ready as output
  {
    pinMode(current_pin, OUTPUT);
    analogWrite(current_pin, 255);
    //  delay(50);
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int current_pin = 54; current_pin <= 63; current_pin++) // Get all the pins ready as output
  {
    pinMode(current_pin, OUTPUT);
    digitalWrite(current_pin, HIGH);
    // delay(50);
  }

  delay(1000);
  for (int current_pin = 54; current_pin <= 63; current_pin++) // Get all the pins ready as output
  {
    digitalWrite(current_pin, LOW);
  }
  for (int current_pin = 4; current_pin <= 12; current_pin++) // Get all the pins ready as output
  {

    analogWrite(current_pin, 0);
    // delay(50);
  }

  digitalWrite(13, LOW);

  for (int i = 20; i <= 32; i++)
  {
    pinMode(i, INPUT_PULLUP);
    lastButtonStates[i] = 0;
  }
  delay(100);
  disp_IAS.clear();
  disp_CRS_L.clear();
  disp_HDG.clear();
}