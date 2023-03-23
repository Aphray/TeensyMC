#pragma once

#include <TeensyTimerTool.h>

#include "stepper.h"
#include "../config.h"
#include "accelerator/accelerator.h"
#include "../communication/enum_factory.h"
#include "../communication/message_agent.h"


#define STEPPER_STATES(X)   \
    X(IDLE)                 \
    X(FAULT)                \
    X(HOMING)               \
    X(PROBING)              \
    X(JOGGING)              \
    X(MOVING)               \
    X(HOLDING)              \
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

        // initiate a move with desired speed (in units/sec) and acceleration (in units/sec)
        void start_move(float speed, float accel);

        // start a jog movement with desired speed (in units/sec) and acceleration (in units/sec)
        void start_jogging(float* unit_vectors, float speed, float accel);

        // stops the jog movement (with deceleration)
        void stop_jogging();

        // home a stepper
        void start_home(uint8_t axis, float speed, float accel);
        void start_home(Stepper* stepper, float speed, float accel);

        // probe a stepper
        void start_probe(uint8_t axis, float speed, float accel, int8_t dir);
        void start_probe(Stepper* stepper, float speed, float accel, int8_t dir);

        // zero a stepper
        void zero_stepper(uint8_t axis);
        void zero_stepper(Stepper* stepper);

        // controlled stop (with deceleration)
        void stop();

        // immediate stop (emergency stop); no deceleration
        void halt();

        // clears a fault condition
        void clear_fault();

        // begins a hold condition
        void hold(uint32_t milliseconds);

        // cancels/clears the hold condition
        void clear_hold();

        // returns the remaining hold time (in milliseconds)
        uint32_t get_hold_time();

        // returns whether the steppers are currently running
        bool steppers_active();

        // returns if the hold condition is set
        bool steppers_holding();

        // returns whether the steppers were homed
        bool steppers_homed();

        // returns whether the steppers are currently accelerating
        bool steppers_accelerating();

        // posts the stepper status info to the serial output
        // if 'queue' is set to true, the message is placed in the message queue and posted from the main loop (for usage inside ISR)
        void post_steppers_status(bool queue = false);

        // returns the speed (in steps/sec) of the accelerator
        float get_accelerator_speed();

        // returns the stepper count
        uint8_t get_num_steppers();

        // returns the master stepper (the one w/ the largest distance to travel)
        Stepper* get_master_stepper();

        // returns the stepper which caused a fault
        Stepper* get_faulted_stepper();

        // get the stepper by axis number (starting from 0)
        Stepper* get_stepper(uint8_t axis);

        // iterator to get the steppers; returns nullptr on complete (after last stepper is retrieved)
        Stepper* get_next_stepper();

        // returns all of the registered steppers
        Stepper** get_all_steppers();

    private:
        StepperState state;
        StepperState prev_state;

        Stepper* fault_stepper;
        Stepper* master_stepper;
        Stepper* steppers[MAX_STEPPERS + 1];
        Stepper* steppers_sort[MAX_STEPPERS + 1];

        uint8_t num_steppers;

        _accelerator accelerator;

        OneShotTimer pulse_timer;
        PeriodicTimer step_timer;
        PeriodicTimer hold_timer;

        // does a single step of the Bresenham algorithm
        bool do_bresenham_step() __always_inline;

        void finish_move() __always_inline;

        // ISRs for the stepping and pulsing
        void step_ISR();
        void pulse_ISR();

        uint32_t hold_ms;

        void hold_ISR();

        void run_steppers(float speed, float accel);

        void change_state(StepperState state) __always_inline;
        void restore_state();
};

inline uint32_t _stepper_control::get_hold_time() {
    return hold_ms;
}

inline bool _stepper_control::steppers_active() {
    return (state == MOVING || state == PROBING || state == HOMING || state == JOGGING);
}

inline bool _stepper_control::steppers_holding() {
    return state == HOLDING;
}

inline bool _stepper_control::steppers_accelerating() {
    return accelerator.accelerating();
}

inline bool _stepper_control::do_bresenham_step() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        if (!(*stepper)->step(master_stepper)) {
            fault_stepper = *stepper;
            return false;
        }
        stepper++;
    }

    return true;
}

inline void _stepper_control::finish_move() {
    step_timer.stop();
    accelerator.reset();
    post_steppers_status(true);

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->finish_move();
        stepper++;
    }
}

inline void _stepper_control::change_state(StepperState new_state) {
    prev_state = state;
    state = new_state;
}

inline void _stepper_control::restore_state() {
    state = prev_state;
}

extern _stepper_control TMCStepperControl;