#include "ServoModule.h"

// Hier maken we het object daadwerkelijk aan
Servo myServo; 

void initServo() {
    myServo.attach(47); // Hier hoort de pinkoppeling thuis!
    myServo.write(90);  // Hier hoort de ruststand thuis!
}
