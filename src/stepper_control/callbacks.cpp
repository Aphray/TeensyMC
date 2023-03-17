
#include "callbacks.h"
#include "stepper_control.h"
#include "../communication/message_agent.h"


inline void CHECK_ACTIVE(char* cmd) {
    if (TMCStepperControl.steppers_active()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers are active", cmd); }
}


inline void CHECK_HOMED(char* cmd) {
    if (!TMCStepperControl.steppers_homed()) { TMCMessageAgent.post_message(ERROR, "Command <%s> cannot run; steppers not homed", cmd); }
}

CALLBACK(EN) {
    char* ax_c = args->next();
    char* en_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i > TMCStepperControl.get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    } 

    int en_i = 0;
    if (!argtoi(en_c, &en_i)) {
        ARG_ERROR(en_c);
        return;
    }

    TMCStepperControl.get_stepper(ax_i)->enable(en_i > 0 ? true : false);
}


CALLBACK(MVE) {
    // move command

    CHECK_ACTIVE(cmd);
    CHECK_HOMED(cmd);

    for (uint8_t n = 0; n < TMCStepperControl.get_num_steppers(); n ++) {

        Stepper* stepper = TMCStepperControl.get_stepper(n);

        char* pos_c = args->next();
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
    float speed_f = 0;

    if (!argtof(speed_c, &speed_f)) {
        ARG_ERROR(speed_c);
        return;
    }

    char* accel_c = args->next();
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
    if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i >= TMCStepperControl.get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    }

    TMCStepperControl.zero_stepper(ax_i);
}

CALLBACK(STP) {
    // stop (controlled w/ deceleration) command

    TMCStepperControl.stop();
}

CALLBACK(HLT) {
    // halt/e-stop command

    TMCStepperControl.halt();
}

CALLBACK(LIM) {
    // set limits command

    char* ax_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i >= TMCStepperControl.get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    }

    Stepper* stepper = TMCStepperControl.get_stepper(ax_i);

    for (uint8_t n = 0; n < 2; n++) {
        char* lim_c = args->next();

        float lim_f = 0;

        switch (*lim_c) {
            case 'R':
                if (!argtof(lim_c + 1, &lim_f)) {
                    ARG_ERROR(lim_c);
                    return;
                }   

                if (n == 0) {
                    stepper->set_min_travel(stepper->get_position() + lim_f);
                } else {
                    stepper->set_max_travel(stepper->get_position() + lim_f);
                }
                break;

            case 'A':
                if (!argtof(lim_c + 1, &lim_f)) {
                    ARG_ERROR(lim_c);
                    return;
                }

                if (n == 0) {
                    stepper->set_min_travel(lim_f);
                } else {
                    stepper->set_max_travel(lim_f);
                }
                break;
            
            case ARG_SKIP_CHAR:
                break;

            default:
                ARG_ERROR(lim_c);
                return;
        }
    }
}