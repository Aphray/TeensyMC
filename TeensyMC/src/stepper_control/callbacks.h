#pragma once

#include "stepper_control.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"

#define CALLBACK(name) inline void name##__cb(char* cmd, ArgList* args)

inline void CHECK_ACTIVE(char* cmd) {
    if (TMCStepperControl.steppers_active()) { 
        TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers are active", cmd); 
        return; 
    }
}

inline void CHECK_HOMED(char* cmd) {
    if (!TMCStepperControl.steppers_homed()) {
        TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers not homed", cmd); 
        return; 
    }
}

inline void ARG_ERROR(char* arg) {
    TMCMessageAgent.post_message(ERROR, "Command error; invalid argument (%s)", arg);
}

CALLBACK(MVE) {
    // move command
    CHECK_ACTIVE(cmd);
    CHECK_HOMED(cmd);

    for (uint8_t n = 0; n < TMCStepperControl.get_num_steppers(); n ++) {
        char* pos_c = args->next();
        Stepper* stepper = TMCStepperControl.get_stepper(n);

        float pos_f = 0;
        switch (*pos_c) {
            case 'A':
                if (!argtof(pos_c + 1, &pos_f)) {
                    ARG_ERROR(pos_c + 1);
                    return;
                }
                stepper->set_target_abs(pos_f);
                break;
            
            case 'R':
                if (!argtof(pos_c + 1, &pos_f)) {
                    ARG_ERROR(pos_c + 1);
                    return;
                }
                stepper->set_target_rel(pos_f);
                break;
                
            case ARG_SKIP_CHAR:
                break;

            default:
                ARG_ERROR(pos_c);
                return;
        }
    }

    char* speed_c = args->next();
    char* accel_c = args->next();

    float speed_f = 0;
    if (!argtof(speed_c, &speed_f)) {
        ARG_ERROR(speed_c);
        return;
    }

    float accel_f = 0;
    if (!argtof(accel_c, &accel_f)) {
        ARG_ERROR(accel_c);
        return;
    }

    TMCStepperControl.start_move(speed_f, accel_f);
}

CALLBACK(PRB) {
    // probe command
    CHECK_ACTIVE(cmd);
    CHECK_HOMED(cmd);

    char* ax_c = args->next();
    char* dir_c = args->next();
    char* speed_c = args->next();
    char* accel_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i)) {
        ARG_ERROR(ax_c);
        return;
    }

    int dir_i = 0;
    if (!argtoi(dir_c, &dir_i)) {
        ARG_ERROR(dir_c);
        return;
    }

    float speed_f = 0;
    if (!argtof(speed_c, &speed_f)) {
        ARG_ERROR(speed_c);
        return;
    }

    float accel_f = 0;
    if (!argtof(accel_c, &accel_f)) {
        ARG_ERROR(accel_c);
        return;
    }

    TMCStepperControl.start_probe(ax_i, speed_f, accel_f, dir_i);
}

CALLBACK(HME) {
    // home command
    CHECK_ACTIVE(cmd);

    char* ax_c = args->next();
    char* speed_c = args->next();
    char* accel_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0) {
        ARG_ERROR(ax_c);
        return;
    }

    float speed_f = 0;
    if (!argtof(speed_c, &speed_f)) {
        ARG_ERROR(speed_c);
        return;
    }

    float accel_f = 0;
    if (!argtof(accel_c, &accel_f)) {
        ARG_ERROR(accel_c);
        return;
    }

    TMCStepperControl.start_home(ax_i, speed_f, accel_f);
}

CALLBACK(FLT) {
    // clear fault command
    CHECK_ACTIVE(cmd);

    TMCStepperControl.clear_fault();
}

CALLBACK(ZRO) {
    // set zero command
    CHECK_ACTIVE(cmd);

    char* ax_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0) {
        ARG_ERROR(ax_c);
        return;
    }

    TMCStepperControl.zero_stepper(ax_i);
}

CALLBACK(STP) {
    // stop command
    TMCStepperControl.stop();
}

CALLBACK(HLT) {
    // halt/e-stop command
    TMCStepperControl.halt();
}