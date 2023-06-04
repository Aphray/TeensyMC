
#include "callbacks.h"
#include "stepper_control.h"

using namespace TeensyMC;
using namespace TeensyMC::SerialComm;

#define ASSERT_INACTIVE if (StepperControl::steppers_active()) { SerialComm::post_message(ERROR, "Command <%s> cannot run; steppers are active", cmd); return; }
#define ASSERT_ACTIVE if (!StepperControl::steppers_active()) { SerialComm::post_message(ERROR, "Command <%s> cannot run; steppers are not active", cmd); return; }
#define ASSERT_HOMED if (!StepperControl::steppers_homed()) { SerialComm::post_message(ERROR, "Command <%s> cannot run; steppers not homed", cmd); return; }


inline void ARG_ERROR(char* arg) {
    SerialComm::post_message(ERROR, "Command error; invalid argument (%s)", arg);
}


bool argtoi(char* arg, int* res) {
    int8_t sign = 1;

    if (arg == nullptr) { return false; }

    for (uint8_t n = 0; n < strlen(arg); n++) {
        switch (arg[n]) {
            case '-':
            case '+':
                if (n != 0) { return false; }
                sign = (arg[n] == '+') ? 1 : -1;
                break;
            case '.':
                return false;
            default:
                if (!isdigit(arg[n])) { return false; }
                break;
        }
    }

    *res = atoi(arg);
    return true;

}

bool argtof(char* arg, float* res) {
    int8_t sign = 1;
    uint8_t decimals = 0;

    if (arg == nullptr) { return false; }

    for (uint8_t n = 0; n < strlen(arg); n++) {
        switch (arg[n]) {
            case '-':
            case '+':
                if (n != 0) { return false; }
                sign = (arg[n] == '+') ? 1 : -1;
                break;
            case '.':
                if (++decimals > 1) { return false; }
                break;
            default:
                if (!isdigit(arg[n])) { return false; }
                break;
        }
    }

    *res = atof(arg);
    return true;
}

CALLBACK(ENABL) {
    // enable command

    char* ax_c = args->next();
    char* en_c = args->next();

    int ax_i = 0;
    bool en_all = false;

    if (ax_c[0] == ARG_SKIP_CHAR) { 
        en_all = true; 
    } else if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i > StepperControl::get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    } 

    int en_i = 0;
    if (!argtoi(en_c, &en_i)) {
        ARG_ERROR(en_c);
        return;
    }

    if (en_all) {
        for (uint8_t n = 0; n < StepperControl::get_num_steppers(); n ++) {
            StepperControl::get_stepper(n)->enable(en_i > 0 ? true : false);
        }
    } else { 
        StepperControl::get_stepper(ax_i)->enable(en_i > 0 ? true : false); 
    }
}


CALLBACK(MOVE) {
    // move command

    ASSERT_HOMED;
    ASSERT_INACTIVE;

    for (uint8_t n = 0; n < StepperControl::get_num_steppers(); n ++) {

        Stepper* stepper = StepperControl::get_stepper(n);

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

    // char* accel_c = args->next();
    // float accel_f = 0;

    // if (!argtof(accel_c, &accel_f)) {
    //     ARG_ERROR(accel_c);
    //     return;
    // }

    StepperControl::start_move(speed_f);
}

CALLBACK(PROBE) {
    // probe command
    
    ASSERT_HOMED;
    ASSERT_INACTIVE;

    char* ax_c = args->next();
    char* dir_c = args->next();
    char* speed_c = args->next();
    // char* accel_c = args->next();

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

    // float accel_f = 0;
    // if (!argtof(accel_c, &accel_f)) {
    //     ARG_ERROR(accel_c);
    //     return;
    // }

    StepperControl::start_probe(ax_i, speed_f, dir_i);
}

CALLBACK(HOME) {
    // home command
    
    ASSERT_INACTIVE;

    char* ax_c = args->next();
    char* speed_c = args->next();
    // char* accel_c = args->next();

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

    // float accel_f = 0;
    // if (!argtof(accel_c, &accel_f)) {
    //     ARG_ERROR(accel_c);
    //     return;
    // }

    StepperControl::start_home(ax_i, speed_f);
}

CALLBACK(CFAULT) {
    // clear fault command

    ASSERT_INACTIVE;

    StepperControl::clear_fault();
}

CALLBACK(ZERO) {
    // set zero command

    ASSERT_INACTIVE;

    char* ax_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i >= StepperControl::get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    }

    StepperControl::zero_stepper(ax_i);
}

CALLBACK(STOP) {
    // stop (controlled w/ deceleration) command

    ASSERT_ACTIVE;
    
    SerialComm::clear_command_queue();
    StepperControl::stop();
}

CALLBACK(HALT) {
    // halt/e-stop command

    ASSERT_ACTIVE;

    SerialComm::clear_command_queue();
    StepperControl::halt();
}

CALLBACK(LIMIT) {
    // set limits command

    char* ax_c = args->next();

    int ax_i = 0;
    if (!argtoi(ax_c, &ax_i) || ax_i < 0 || ax_i >= StepperControl::get_num_steppers()) {
        ARG_ERROR(ax_c);
        return;
    }

    Stepper* stepper = StepperControl::get_stepper(ax_i);

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

CALLBACK(JOG) {
    // jog begin command

    ASSERT_INACTIVE;
    ASSERT_HOMED;

    float sum_sqr;
    float unit_vectors[StepperControl::get_num_steppers()] = {0};

    for (uint8_t s = 0; s < 2; s++) {
        for (uint8_t n = 0; n < StepperControl::get_num_steppers(); n ++) {
            if (s == 0) {
                char* pos_c = args->next();
                float pos_f = 0;

                if (*pos_c == ARG_SKIP_CHAR) {
                    continue;
                } else if (!argtof(pos_c, &pos_f)) { 
                    ARG_ERROR(pos_c);
                    return;
                }
                unit_vectors[n] = pos_f;
                sum_sqr += (pos_f * pos_f);
            } else {
                // compute actual unit vector (u / |u|)
                unit_vectors[n] /= sqrtf(sum_sqr);
            }
        }
    }

    char* speed_c = args->next();
    float speed_f = 0;

    if (!argtof(speed_c, &speed_f)) {
        ARG_ERROR(speed_c);
        return;
    }

    // char* accel_c = args->next();
    // float accel_f = 0;

    // if (!argtof(accel_c, &accel_f)) {
    //     ARG_ERROR(accel_c);
    //     return;
    // }

    StepperControl::start_jogging(unit_vectors, speed_f);
}


CALLBACK(JOGC) {
    StepperControl::stop_jogging();
}


CALLBACK(HOLD) {

    ASSERT_INACTIVE;
    ASSERT_HOMED;

    char* ms_c = args->next();
    int ms_i = 0;
    
    if (!argtoi(ms_c, &ms_i) || ms_i <= 0) {
        ARG_ERROR(ms_c);
        return;
    }

    StepperControl::hold(ms_i);
}


CALLBACK(HOLDC) {
    StepperControl::clear_hold();
}

CALLBACK(STORE) {
    ASSERT_INACTIVE;
    ASSERT_HOMED;

    char* idx_c = args->next();
    int idx_i = 0;

    if (!argtoi(idx_c, &idx_i) || idx_i < 0) {
        ARG_ERROR(idx_c);
        return;
    }

    StepperControl::store_position(idx_i);
}

CALLBACK(RECALL) {
    ASSERT_INACTIVE;
    ASSERT_HOMED;

    char* idx_c = args->next();
    int idx_i = 0;

    if (!argtoi(idx_c, &idx_i) || idx_i < 0) {
        ARG_ERROR(idx_c);
        return;
    }

    StepperControl::recall_position(idx_i);
}