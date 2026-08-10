#pragma once
#include <Arduino.h>
#include <CmdMessenger.h>

extern CmdMessenger messenger;

extern bool isReady;
extern bool isStarted;


extern int olddimvalue ;
extern int GameState ;
extern int DimState ;
extern int lastButtonStates[];
extern int buttonStates[];


void initHardware();