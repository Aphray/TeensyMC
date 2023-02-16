#pragma once

#include "stepper_control.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"

#define CALLBACK(name) inline void name##__cb(char* cmd, ArgList* args)

#define CHECK_ACTIVE if (TMCStepperControl.steppers_active()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers are active", cmd); return; }
#define CHECK_HOMED if (!TMCStepperControl.steppers_homed()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers not homed", cmd); return; }
#define ARG_ERROR TMCMessageAgent.post_message(ERROR, "Command error; invalid argument")

CALLBACK(MVE) {
    // move command
    CHECK_ACTIVE;
    CHECK_HOMED;

    for (uint8_t n = 0; n < TMCStepperControl.get_num_steppers(); n ++) {
        char* pos_c = args->next();
        Stepper* stepper = TMCStepperControl.get_stepper(n);

        float pos_f = 0;
        switch (*pos_c) {
            case 'A':
                stepper->set_target_abs_units(pos_f);
                break;
            
            case 'R':
            case ARG_SKIP_CHAR:
                stepper->set_target_rel_units(pos_f);
                break;

            default:
                ARG_ERROR;
                return;
        }
    }
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