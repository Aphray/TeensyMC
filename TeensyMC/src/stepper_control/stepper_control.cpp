#include <algorithm>

#include "stepper.h"
#include "stepper_control.h"
#include "callbacks.h"
#include "../communication/enum_factory.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"



_stepper_control::_stepper_control() {
    num_steppers = 0;
    state = HOME_STEPPERS_FIRST ? HOME_FIRST : IDLE;
}

void _stepper_control::begin() {

    // setup the timers
    pulse_timer.begin([this]{ this->pulse_ISR(); });
    step_timer.begin([this]{ this->step_ISR(); }, 1000, false);

    // add the serial commands
    TMCSerialCommand.register_command("MVE", 2, &num_steppers);
    TMCSerialCommand.register_command("PRB", 1);
    TMCSerialCommand.register_command("HME", 1);
    TMCSerialCommand.register_command("STP", 0);
    TMCSerialCommand.register_command("HLT", 0);
    TMCSerialCommand.register_command("FLT", 0);

    // attach callbacks
    TMCSerialCommand.add_callback("MVE", &MVE__cb);
    TMCSerialCommand.add_callback("PRB", &PRB__cb);
    TMCSerialCommand.add_callback("HME", &HME__cb);
    TMCSerialCommand.add_callback("STP", &STP__cb);
    TMCSerialCommand.add_callback("HLT", &HLT__cb);
    TMCSerialCommand.add_callback("FLT", &FLT__cb);
}


void _stepper_control::sort_steppers() {
    std::sort(steppers_sort, steppers_sort + MAX_STEPPERS, Stepper::cmp_delta);
    master = steppers_sort[0];
}

void _stepper_control::add_stepper(Stepper& stepper) {

    if (++num_steppers > MAX_STEPPERS) {
        TMCMessageAgent.post_message(ERROR, "Too many steppers initialized; 'MAX_STEPPERS' is set to %i", MAX_STEPPERS);
        abort();
    }

    steppers[num_steppers - 1] = &stepper;
    steppers_sort[num_steppers - 1] = &stepper;
}

void _stepper_control::start_move(float speed, float accel) {
    if (!master->move_complete()) { return; }

    float start_speed = 0;

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->prepare(master, &start_speed, &speed, &accel);
        stepper++;
    }

    
    if (state != HOMING || state != PROBING) {
        state = ACTIVE;
        TMCMessageAgent.post_message(INFO, "Move started");
    }

    accelerator.prepare(master->get_delta(), start_speed, speed, accel);
    step_timer.setPeriod(1);
    step_timer.start();
}

void _stepper_control::home(uint8_t axis) {
    if (axis > num_steppers) { return; }
    _stepper_control::home(steppers[axis]);
}

void _stepper_control::home(Stepper* stepper) {
    if (stepper == nullptr) { return; }

    Stepper** other_stepper = steppers;
    while (*other_stepper) {
        if (*other_stepper != stepper) {
            (*other_stepper)->set_target_rel(0);
        } else {

        }
    }
}

bool _stepper_control::steppers_active() {
    return (state == ACTIVE || state == PROBING || state == HOMING);
}

bool _stepper_control::steppers_homed() {
    Stepper** stepper = steppers;
    while (*stepper) {
        if (!(*stepper)->is_homed()) { return false; }
        stepper++;
    }
    return true;
}

void _stepper_control::post_steppers_status(bool queue) {
    static const char* const STEPPER_STATE_STRINGS[] = { STEPPER_STATES(MAKE_STRINGS) };

    char message[MESSAGE_BUFFER_SIZE];
    sprintf(message, "(%s)", STEPPER_STATE_STRINGS[state]);

    Stepper** stepper = steppers;
    while (*stepper) {
        sprintf(message + strlen(message), " AX%i:(%f,%f)", 
            (*stepper)->get_axis_id(), (*stepper)->get_position_in_units(), (*stepper)->get_speed_in_units());

        stepper++;
    }

    if (queue) { TMCMessageAgent.queue_message(STATUS, message); } 
    else { TMCMessageAgent.post_message(STATUS, message); }
}

float _stepper_control::get_accelerator_speed() {
    return accelerator.current_speed;
}

uint8_t _stepper_control::get_num_steppers() {
    return num_steppers;
}

Stepper* _stepper_control::get_master_stepper() {
    return master;
}

Stepper* _stepper_control::get_stepper(uint8_t axis) {
    return (axis < num_steppers) ? steppers[axis] : nullptr;
}

Stepper* _stepper_control::get_next_stepper() {
    static uint8_t idx = 0;
    if (idx == num_steppers) {
        idx = 0;
        return nullptr;
    } else {
        return steppers[idx++];
    }
}

Stepper** _stepper_control::get_all_steppers() {
    return steppers;
}

void _stepper_control::step_ISR() {

    switch (state) {
        case HOME_FIRST:
        case FAULT:
        case IDLE:
            step_timer.stop();
            post_steppers_status(true);
            break;

        case ACTIVE:
        case_ACTIVE:
        
            do_bresenham_step();

            if (in_fault) {
                state = FAULT;
                step_timer.stop();
                TMCMessageAgent.queue_message(CRITICAL, "Fault: software limit on axis %d", in_fault->get_axis_id());
                post_steppers_status(true);

            } else if (master->move_complete()) {
                state = IDLE;
                step_timer.stop();
                accelerator.current_speed = 0;
                TMCMessageAgent.queue_message(INFO, "Move complete");
                post_steppers_status(true);

            } else {
                step_timer.setPeriod(accelerator.compute_next_step_period());
            }

            pulse_timer.trigger(PULSE_WIDTH_US);
    }
}

void _stepper_control::pulse_ISR() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        (*stepper)->clear_step();
        stepper++;
    }
}