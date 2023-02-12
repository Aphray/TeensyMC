#pragma once

#include <TeensyTimerTool.h>

#include "stepper.h"
#include "../default_config.h"
#include "accelerators/accelerator.h"
#include "../communication/enum_factory.h"

#define STEPPER_STATES(X)   \
    X(IDLE)                 \
    X(ACTIVE)               \
    X(FAULT)                \
    X(HOMING)               \
    X(PROBING)              \
    X(HOME_FIRST)           \


enum StepperState {
    STEPPER_STATES(MAKE_ENUM)
};

using namespace TeensyTimerTool;

class _stepper_control {

    public:
        StepperState state;

        _stepper_control();

        void setup_timers();

        void sort_steppers();
        
        void add_stepper(Stepper* stepper, uint8_t axis);

        void start_move(float speed, float accel);

        bool steppers_active();

        void post_steppers_status();

    private:

        Stepper* master;
        Stepper* in_fault;
        Stepper* steppers[STEPPERS + 1];
        Stepper* steppers_sort[STEPPERS + 1];

        OneShotTimer pulse_timer;
        PeriodicTimer step_timer;

        void do_bresenham_step() __always_inline;

        void step_ISR();
        void pulse_ISR();

};

inline void _stepper_control::do_bresenham_step() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        if (!(*stepper)->step(master)) {
            in_fault = *stepper;
            return;
        }
        stepper++;
    }
}

extern _stepper_control TMCStepperControl;