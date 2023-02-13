#include "TeensyMC.h"

// create some steppers
Stepper xstepper(1, 2); // (<STEP_PIN>,<DIR_PIN>)
Stepper ystepper(3, 4);
Stepper zstepper(5, 6);


void foo(char* cmd, ArgList* args) {
    TMCMessageAgent.post_message(DEBUG, "cmd=%s", cmd);
    TMCMessageAgent.post_message(DEBUG, "arg1=%s", args->next());
}


void setup() {
    // add the steppers to the registry
    TMCStepperControl.add_stepper(&xstepper);
    TMCStepperControl.add_stepper(&ystepper);
    TMCStepperControl.add_stepper(&zstepper);

    // run the setup/initialization
    // IMPORTANT: must be called __AFTER__ adding the steppers
    TMCSetup();

    // add custom serial commands
    TMCSerialCommand.register_command("FOO", 1);    // (<CMD_NAME>,<REQUIRED_NUMBER_OF_ARGUMENTS>)
    TMCSerialCommand.add_callback("FOO", &foo);     // (<CMD_NAME>,<CALLBACK>)
}


void loop() {
    // run TeensyMC
    TMCRun();

    // do other __NON-BLOCKING__ stuff here...
}