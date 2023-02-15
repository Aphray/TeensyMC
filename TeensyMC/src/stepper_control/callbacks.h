#pragma once

#include "stepper_control.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"

#define CALLBACK(name) inline void name##__cb(char* cmd, ArgList* args)

#define CHECK_ACTIVE if (TMCStepperControl.steppers_active()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers are active", cmd); return; }
#define CHECK_HOMED if (!TMCStepperControl.steppers_homed()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers not homed", cmd); return; }


CALLBACK(MVE) {
    // move command
    CHECK_ACTIVE;
    CHECK_HOMED;

}

CALLBACK(PRB) {
    // probe command
    CHECK_ACTIVE;
    CHECK_HOMED;

}

CALLBACK(HME) {
    // home command
    CHECK_ACTIVE;

}

CALLBACK(FLT) {
    // clear fault command
    CHECK_ACTIVE;

}

CALLBACK(ZRO) {
    // set zero command
    CHECK_ACTIVE;

}

CALLBACK(STP) {
    // stop command

}

CALLBACK(HLT) {
    // halt/e-stop command

}