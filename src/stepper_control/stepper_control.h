#pragma once

#include "stepper.h"
#include "../communication/enum_factory.h"

#define STEPPER_STATES(X)   \
    X(IDLE)                 \
    X(FAULT)                \
    X(HOMING)               \
    X(PROBING)              \
    X(JOGGING)              \
    X(MOVING)               \
    X(HOLDING)              \
    X(HOME_FIRST)           \


namespace TeensyMC { namespace StepperControl {

    enum StepperState {
        STEPPER_STATES(MAKE_ENUM)
    };
    
    namespace internal {
        // initialization
        void begin();

        // sorts the steppers by distance (largest -> smallest)
        void sort_steppers();

        // posts the stepper status info to the serial output
        // if 'queue' is set to true, the message is placed in the message queue and posted from the main loop (for usage inside ISR)
        void post_steppers_status(bool queue = false);

        // stepper processing loop (homing, probing, etc.)
        void process();

    } // namespace internal

    bool check_state(StepperState state);

    template<typename... StepperStates>
    bool check_state(StepperStates... states) {
        for (StepperState state: {states...}) {
            if (check_state(state)) return true;
        }
        return false;
    }

    // add a stepper
    void add_stepper(Stepper& stepper);

    // initiate a move with desired speed (in units/sec) and acceleration (in units/sec)
    void start_move(float speed);

    // start a jog movement with desired speed (in units/sec) and acceleration (in units/sec)
    void start_jogging(float* unit_vectors, float speed);
    
    // stops the jog movement (with deceleration)
    void stop_jogging();

    // home a stepper
    void start_home(uint8_t axis, float speed);
    void start_home(Stepper* stepper, float speed);

    // probe a stepper
    void start_probe(uint8_t axis, float speed, int8_t dir);
    void start_probe(Stepper* stepper, float speed, int8_t dir);

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

    // returns whether the steppers were homed
    bool steppers_homed();

    // returns the speed (in steps/sec) of the accelerator
    float get_accelerator_speed();

    // returns in the steppers are accelerating/decelerating
    bool steppers_accelerating();

    // returns the stepper count
    uint8_t get_num_steppers();

    // returns the master stepper (the one w/ the largest distance to travel)
    Stepper* get_master_stepper();

    // returns the stepper which caused a fault
    Stepper* get_faulted_stepper();

    // get the stepper by axis number (starting from 0)
    Stepper* get_stepper(uint8_t axis);

    // returns all of the registered steppers
    Stepper** get_all_steppers();

}} // namespace TeensyMC::StepperControl