#include "TeensyMC.h"

// create some steppers
Stepper xstepper(1, 2); // (<STEP_PIN>,<DIR_PIN>)
Stepper ystepper(3, 4);
// Stepper zstepper(5, 6);

void foo() {
    TMCMessageAgent.post_message(DEBUG, "FOO");
}

void setup() {
    // run the setup/initialization
    TMC_Begin();

    xstepper.set_units_per_step(0.0015625);
    xstepper.set_max_accel(500);
    xstepper.set_speed_limits(0.01, 20);
    xstepper.set_min_max_travel(-200, 200);

    ystepper.set_units_per_step(0.0015625);
    ystepper.set_max_accel(500);
    ystepper.set_speed_limits(0.01, 20);
    ystepper.set_min_max_travel(-200, 200);


    TMCStepperControl.add_stepper(xstepper);
    TMCStepperControl.add_stepper(ystepper);
    // TMCStepperControl.add_stepper(zstepper);

    TMCEventManager.attach_callback(MOVE_COMPLETE, &foo);
}


void loop() {
    // run TeensyMC
    TMC_Run();

    // do other __NON-BLOCKING__ stuff here...
}