#include <algorithm>

#include "stepper.h"
#include "stepper_control.h"
#include "../communication/enum_factory.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"



_stepper_control::_stepper_control() {
    state = HOME_STEPPERS_FIRST ? HOME_FIRST : IDLE;
}


void _stepper_control::sort_steppers() {
    std::sort(steppers_sort, steppers_sort + STEPPERS, Stepper::cmp_delta);
    master = steppers_sort[0];
}

void _stepper_control::add_stepper(Stepper* stepper, uint8_t axis) {
    steppers[axis] = stepper;
    steppers_sort[axis] = stepper;
}

void _stepper_control::start_move(float speed, float accel) {
    
}

void _stepper_control::setup_timers() {
    pulse_timer.begin([this]{ this->pulse_ISR(); });
    step_timer.begin([this]{ this->step_ISR(); }, 1000, false);
}

void _stepper_control::post_steppers_status() {
    static const char* const STEPPER_STATE_STRINGS[] = { STEPPER_STATES(MAKE_STRINGS) };

    char message[MESSAGE_BUFFER_SIZE];
    float acc_speed = accelerator.current_speed;

    sprintf(message, "(%s)", STEPPER_STATE_STRINGS[state]);
    for (uint8_t n = 0; n < STEPPERS; n++) {
        Serial.println(STEPPERS);
        Stepper* stepper = steppers[n];
        sprintf(message + strlen(message), " AX%i:(%f,%f)", n, stepper->position, acc_speed);
    }

    TMCMessageAgent.post_message(STATUS, message);
}

bool _stepper_control::steppers_active() {
    return (state == ACTIVE || state == PROBING || state == HOMING);
}

void _stepper_control::step_ISR() {

    switch (state) {
        case HOME_FIRST:
        case FAULT:
        case IDLE:
            step_timer.stop();
            break;

        case ACTIVE:
        case_ACTIVE:
        
            do_bresenham_step();

            if (in_fault) {
                state = FAULT;
                step_timer.stop();
                TMCMessageAgent.queue_message(CRITICAL, "Fault: software limit on axis %d", in_fault->axis);

            } else if (master->move_complete()) {
                state = IDLE;
                step_timer.stop();
                accelerator.current_speed = 0;
                TMCMessageAgent.queue_message(INFO, "Move complete");

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