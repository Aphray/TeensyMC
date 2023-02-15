#pragma once

#include <TeensyTimerTool.h>

#include "stepper.h"
#include "../TMC_default_config.h"
#include "accelerator/accelerator.h"
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
        _stepper_control();

        // initialization
        void begin();

        // sorts the steppers by distance (largest -> smallest)
        void sort_steppers();
        
        // add a stepper
        void add_stepper(Stepper& stepper);

        // initiate a move
        void start_move(float speed, float accel);

        // returns whether the steppers are currently runnin
        bool steppers_active();

        // returns whether the steppers were homed
        bool steppers_homed();

        // posts the stepper status info to the serial output
        // if 'queue' is set to true, the message is placed in the message queue and posted from the main loop (for usage inside ISRs)
        void post_steppers_status(bool queue);

        // returns the speed (in steps/sec) of the accelerator
        float get_accelerator_speed();

        // returns the stepper count
        uint8_t get_num_steppers();

        // returns the master stepper (largest distance to travel)
        Stepper* get_master_stepper();

    private:
        StepperState state;

        Stepper* master;
        Stepper* in_fault;
        Stepper* steppers[MAX_STEPPERS + 1];
        Stepper* steppers_sort[MAX_STEPPERS + 1];

        uint8_t num_steppers;

        _accelerator accelerator;

        OneShotTimer pulse_timer;
        PeriodicTimer step_timer;

        // does a single step of the Bresenham algorithm
        void do_bresenham_step() __always_inline;

        // ISRs for the stepping and pulsing
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