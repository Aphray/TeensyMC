#include <algorithm>

#include "stepper_control.h"
#include "callbacks.h"
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
    TMCSerialCommand.register_command("PRB", 4);
    TMCSerialCommand.register_command("HME", 3);
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
    master_stepper = steppers_sort[0];
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
    if (steppers_active()) { return; }

    state = ACTIVE;
    run_steppers(speed, accel);
}

void _stepper_control::start_home(uint8_t axis, float speed, float accel) {
    if (axis > num_steppers) { return; }
    _stepper_control::start_home(steppers[axis], speed, accel);
}

void _stepper_control::start_home(Stepper* stepper, float speed, float accel) {
    if (stepper == nullptr || steppers_active()) { return; }
    state = HOMING;
    stepper->prepare_homing();
    run_steppers(speed, accel);
}

void _stepper_control::start_probe(uint8_t axis, float speed, float accel, int8_t dir) {
    if (axis > num_steppers) { return; }
    _stepper_control::start_probe(steppers[axis], speed, accel, dir);
}

void _stepper_control::start_probe(Stepper* stepper, float speed, float accel, int8_t dir) {
    if (stepper == nullptr || steppers_active()) { return; }
    state = PROBING;
    stepper->prepare_probing(dir);
    run_steppers(speed, accel);
}

void _stepper_control::zero_stepper(uint8_t axis) {
    if (axis > num_steppers) { return; }
    _stepper_control::zero_stepper(steppers[axis]);
}

void _stepper_control::zero_stepper(Stepper* stepper) {
    stepper->set_zero();
}

void _stepper_control::stop() {
    if (!steppers_active()) { return; }
    accelerator.decelerate_now();
}

void _stepper_control::halt() {
    if (!steppers_active()) { return; }

    state = FAULT;
    finish_move();
}

void _stepper_control::clear_fault() {
    if (state != FAULT) { return; }
    fault_stepper = nullptr;
    state = HOME_STEPPERS_FIRST ? HOME_FIRST : IDLE;
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

void _stepper_control::post_steppers_status(bool queue = false) {
    static const char* const STEPPER_STATE_STRINGS[] = { STEPPER_STATES(MAKE_STRINGS) };

    char message[MESSAGE_BUFFER_SIZE];
    sprintf(message, "(%s)", STEPPER_STATE_STRINGS[state]);

    Stepper** stepper = steppers;
    while (*stepper) {
        sprintf(message + strlen(message), " AX%i:(%f,%f)", 
            (*stepper)->get_axis_id(), (*stepper)->get_position(), (*stepper)->get_speed());

        stepper++;
    }

    if (queue) { TMCMessageAgent.queue_message(STATUS, message); } 
    else { TMCMessageAgent.post_message(STATUS, message); }
}

float _stepper_control::get_accelerator_speed() {
    return accelerator.get_speed();
}

uint8_t _stepper_control::get_num_steppers() {
    return num_steppers;
}

Stepper* _stepper_control::get_master_stepper() {
    return master_stepper;
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
            finish_move();
            break;

        case ACTIVE:
        case_ACTIVE:
        {
            do_bresenham_step();
            float period = accelerator.compute_next_step_period();

            if (state == FAULT) {
                finish_move();

            } else if (master_stepper->move_complete() || period < 0) {
                state = IDLE;
                finish_move();
                TMCMessageAgent.queue_message(INFO, "Move complete");

            } else { step_timer.setPeriod(period); }

            pulse_timer.trigger(PULSE_WIDTH_US);
        }
    }
}

void _stepper_control::pulse_ISR() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        (*stepper)->clear_step();
        stepper++;
    }
}

void _stepper_control::run_steppers(float speed, float accel) {

    float start_speed = 0;
    speed = master_stepper->cvt_to_steps(speed);
    accel = master_stepper->cvt_to_steps(accel);

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->constrain_speed_accel(master_stepper, &start_speed, &speed, &accel);
        stepper++;
    }

    accelerator.prepare(master_stepper->get_delta_steps(), start_speed, speed, accel);
    step_timer.setPeriod(1);
    step_timer.start();
}