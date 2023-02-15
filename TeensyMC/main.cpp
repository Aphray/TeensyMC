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
    // run the setup/initialization
    TMC_Begin();

    TMCStepperControl.add_stepper(xstepper);
    TMCStepperControl.add_stepper(ystepper);
    TMCStepperControl.add_stepper(zstepper);

    TMC_SetupCommands();

    // add custom serial commands
    TMCSerialCommand.register_command("FOO", 1);    // (<CMD_NAME>,<REQUIRED_NUMBER_OF_ARGUMENTS>)
    TMCSerialCommand.add_callback("FOO", &foo);     // (<CMD_NAME>,<CALLBACK>)
}


void loop() {
    // run TeensyMC
    TMC_Run();

    // do other __NON-BLOCKING__ stuff here...
}