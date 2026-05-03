#pragma once

enum
{
   kRequest = 0,     // Request from SPAD.neXt              ... Documentation lists as "Command:0"
  kCommand = 1,     // Command to SPAD.neXt                ... Documentation lists as "Command:1"
  kEvent = 2,       // Events from SPAD.neXt                 ... Documentation lists as "Command:2"
  kDebug = 3,       // Debug strings to SPAD.neXt Logfile    ... Documentation lists as "Debug Channel"
  kSimCommand = 4,  // Send Event to Simulation         ... Documentation lists as "Command:4"
  kData = 5,
  kLed = 6,
  kDisplay = 7,
  kPaneldimmer1 = 20,
  KPanLightDim = 21,
  kGameState = 22,
  dIASoverspeed = 23,   //22
  dIASunderspeed = 24,  //23
  dIASblank = 25,       //22
  dHeading = 26,        //31
  dIASMach = 27,        //32
  dCourseL = 28,        //33
  dHDGBANK = 29,
};