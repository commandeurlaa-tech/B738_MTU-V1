#pragma once
#include <Arduino.h>
#include <CmdMessenger.h>
#include "TM1637TinyDisplay.h"
#include "TM1637TinyDisplay6.h"
#include <Wire.h>  // Library for I2C communicationp
#include <RotaryEncoder.h>

extern CmdMessenger messenger;

extern bool isReady;
extern bool isStarted;
extern bool IAS_blank;
extern bool IASoverspeed;
extern bool IASunderspeed;
extern bool VSblank;
extern int ALTWARN;
extern int HDG_BANK_SEL;
extern char Angel;
extern int HDG_VAL;

extern int olddimvalue ;
extern int GameState ;
extern int DimState ;
extern int lastButtonStates[];
extern int buttonStates[];

extern TM1637TinyDisplay6 disp_IAS;
extern TM1637TinyDisplay disp_CRS_L;
extern TM1637TinyDisplay disp_HDG;

extern RotaryEncoder CRS_L;
extern RotaryEncoder IAS;
extern RotaryEncoder HDG;

extern long position1;
extern long position2;
extern long position3;

void initHardware();