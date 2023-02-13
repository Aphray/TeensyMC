#include "TeensyMC.h"

// create some steppers
Stepper xstepper(1, 2);
Stepper ystepper(3, 4);
Stepper zstepper(5, 6);


void foo(char* cmd, ArgList* args) {
    TMCMessageAgent.post_message(DEBUG, "cmd=%s", cmd);
    TMCMessageAgent.post_message(DEBUG, "arg1=%s", args->next());
}


void setup() {
    // run the setup/initialization
    TMCSetup();

    // add the steppers to the registry
    TMCStepperControl.add_stepper(&xstepper);
    TMCStepperControl.add_stepper(&ystepper);
    TMCStepperControl.add_stepper(&zstepper);

    // add custom serial commands
    TMCSerialCommand.register_command("FOO", 1); // (<CMD_NAME>,<REQUIRED_NUMBER_OF_ARGUMENTS>)
    TMCSerialCommand.add_callback("FOO", &foo);
}


void loop() {
    // run TeensyMC
    TMCRun();

    // do other __NON-BLOCKING__ stuff here...
}