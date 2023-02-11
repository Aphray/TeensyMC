#define STEPPERS 3
#include "TeensyMC.h"


void setup() {
    TMCSetup();
}


void loop() {
    // run TeensyMC
    TMCRun();

    // do other __NON-BLOCKING__ stuff here...
}