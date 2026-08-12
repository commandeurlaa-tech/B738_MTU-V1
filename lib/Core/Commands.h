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
  kBrakeLeft= 23,
  kBrakeRight= 24,
  kParkingBrake = 25,

};