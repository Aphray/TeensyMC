#pragma once

#include "stepper_control/stepper.h"
#include "communication/serial_comm.h"
#include "stepper_control/stepper_control.h"

#define TMC TeensyMC
#define TMCSerialComm TeensyMC::SerialComm
#define TMCStepperCtrl TeensyMC::StepperControl


// namespace TMC = TeensyMC;
// namespace TMCSerialComm = TeensyMC::SerialComm;
// namespace TMCStepperControl = TeensyMC::StepperControl;


namespace TeensyMC {
    // initialize TeensyMC
    void init(bool (*ext_init)() = nullptr);

    // check if initialization was complete
    bool initialized();

    // run the TeensyMC loop
    void run();
}