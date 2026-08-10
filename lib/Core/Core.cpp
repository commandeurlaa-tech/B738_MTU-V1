#include "Core.h"
#include "Commands.h"


CmdMessenger messenger(Serial);

String authkey = "AUTHOR=2656302a-8aa7-4c93-bda6-7a12d883953e";

bool isReady = false;
bool isStarted = false;


int olddimvalue = 0;
int GameState = 0;
int DimState = 0;
int lastButtonStates[26] = {};
int buttonStates[26];


void initHardware()
{

  Serial.begin(115200);

 

  for (int current_pin = 28; current_pin <= 47; current_pin++) // Get all the pins ready as output
  {
    pinMode(current_pin, OUTPUT);
    digitalWrite(current_pin, HIGH);
    //  delay(50);
  }
  

  for (int i = 10; i <= 25; i++)
  {
    pinMode(i, INPUT_PULLUP);
    lastButtonStates[i] = 0;
  }
  delay(100);
  
}