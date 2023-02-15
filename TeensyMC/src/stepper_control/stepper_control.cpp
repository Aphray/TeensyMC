#include <algorithm>

#include "stepper.h"
#include "stepper_control.h"
#include "../communication/enum_factory.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"



_stepper_control::_stepper_control() {
    num_steppers = 0;
    state = HOME_STEPPERS_FIRST ? HOME_FIRST : IDLE;
}


void _stepper_control::sort_steppers() {
    std::sort(steppers_sort, steppers_sort + MAX_STEPPERS, Stepper::cmp_delta);
    master = steppers_sort[0];
}

void _stepper_control::add_stepper(Stepper* stepper) {

    if (++num_steppers > MAX_STEPPERS) {
        TMCMessageAgent.post_message(ERROR, "Too many steppers initialized; 'MAX_STEPPERS' is set to %i", MAX_STEPPERS);
        abort();
    }

    steppers[num_steppers - 1] = stepper;
    steppers_sort[num_steppers - 1] = stepper;
}

void _stepper_control::start_move(float speed, float accel) {
    if (master->move_complete()) return;

    // float start_speed = master->min_speed;

    // if (master->max_accel < accel) accel = master->max_accel;
    // if (master->max_speed < speed) speed = master->max_speed;

    float start_speed = 0;

    Stepper** stepper = steppers_sort;
    while (*stepper) {
        (*stepper)->prepare(master, &start_speed, &speed, &accel);
        stepper++;
    }

    // for (uint8_t n = 1; n < num_steppers; n++) {
    //     Stepper* stepper = steppers_sort[n];
    //     stepper->prepare(master, &speed, &accel);

    //     if (steppers_sort[n]->max_accel < accel) accel = steppers_sort[n]->max_accel;
    //     if (steppers_sort[n]->min_speed > start_speed) start_speed = steppers_sort[n]->min_speed;

    //     float norm = steppers_sort[n]->delta / master->delta;
    //     if (steppers_sort[n]->max_speed < (norm * speed)) speed = steppers_sorted[n]->max_speed / norm;
    // }

    if (state != HOMING || state != PROBING) {
        state = ACTIVE;
        TMCMessageAgent.post_message(INFO, "Move started");
    }

    accelerator.prepare(master->delta, start_speed, speed, accel);
    step_timer.setPeriod(1);
    step_timer.start();
}

void _stepper_control::setup_timers() {
    pulse_timer.begin([this]{ this->pulse_ISR(); });
    step_timer.begin([this]{ this->step_ISR(); }, 1000, false);
}

bool _stepper_control::steppers_active() {
    return (state == ACTIVE || state == PROBING || state == HOMING);
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

    if (queue) {
        TMCMessageAgent.queue_message(STATUS, message);
    } else {
        TMCMessageAgent.post_message(STATUS, message);
    }
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