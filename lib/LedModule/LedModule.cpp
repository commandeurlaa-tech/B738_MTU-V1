#include "Core.h"
#include "Commands.h"

void createLED(int id, const char* name, int cmd)
{
    messenger.sendCmdStart(cmd);
    messenger.sendCmdArg("OUTPUT");
    messenger.sendCmdArg(id);
    messenger.sendCmdArg(name);
    messenger.sendCmdArg("LED");
    messenger.sendCmdArg("SPAD_LED");
    messenger.sendCmdEnd();
}

// ---------- DEFINITIES ----------
struct LedDef {
    int id;
    const char* name;
};

// ---------- LED LIJST ----------
LedDef leds[] = {

    // ========
    {28, "PARKINGBRAKE"},
 
    
};

// ---------- CREATE ALL ----------
void createAllLEDs()
{
    int count = sizeof(leds) / sizeof(leds[0]);

    for (int i = 0; i < count; i++)
    {
        int cmd = (i == 0) ? kRequest : 0;
        createLED(leds[i].id, leds[i].name, cmd);
    }
}


void onDeviceLed() {
  int ledID = messenger.readInt32Arg();                                 // Led ID
  bool ledVal = messenger.readBoolArg();                                // Led value
  (ledVal) ? (digitalWrite(ledID, HIGH)) : (digitalWrite(ledID, LOW));  // was ledpin
}