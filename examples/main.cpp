#include "TeensyMC.h"

const uint8_t ZPROBE_PIN = 10;

const uint8_t XHOME_PIN = 11;
const uint8_t YHOME_PIN = 12;
const uint8_t ZHOME_PIN = 13;

// create some steppers
Stepper xstepper(1, 2); // (<STEP_PIN>,<DIR_PIN>)
Stepper ystepper(3, 4);
Stepper zstepper(5, 6);


int8_t check_zprobe() {
    if (digitalReadFast(ZPROBE_PIN)) { return 1; }
    return 0;
}

int8_t check_homing(const uint8_t pin) {
    if (digitalReadFast(pin)) { return 1; }
    return 0;
}

int8_t (*xhome)() = [&]() { return check_homing(XHOME_PIN); };
int8_t (*yhome)() = [&]() { return check_homing(YHOME_PIN); };
int8_t (*zhome)() = [&]() { return check_homing(ZHOME_PIN); };


void setup() {
    // run the setup/initialization
    TMC_Begin();

    xstepper.set_units_per_step(0.0015625);
    xstepper.set_max_accel(500);
    xstepper.set_speed_limits(0.01, 20);
    xstepper.set_min_max_travel(-200, 200);
    xstepper.set_homing_callback(xhome);

    ystepper.set_units_per_step(0.0015625);
    ystepper.set_max_accel(500);
    ystepper.set_speed_limits(0.01, 20);
    ystepper.set_min_max_travel(-200, 200);
    ystepper.set_homing_callback(yhome);

    zstepper.set_units_per_step(0.0015625);
    zstepper.set_max_accel(500);
    zstepper.set_speed_limits(0.01, 20);
    zstepper.set_min_max_travel(-200, 200);
    zstepper.set_homing_callback(zhome);
    zstepper.set_probing_callback(&check_zprobe);

    TMCStepperControl.add_stepper(xstepper);
    TMCStepperControl.add_stepper(ystepper);
    TMCStepperControl.add_stepper(zstepper);
}


void loop() {
    // run TeensyMC
    TMC_Run();

    // do other __NON-BLOCKING__ stuff here...
}