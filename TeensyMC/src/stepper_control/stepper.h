#pragma once

#include <Arduino.h>
#include "../communication/enum_factory.h"


class Stepper {

    public:
        Stepper(uint8_t step_pin, uint8_t dir_pin);

        // inverts the the HIGH/LOW signals on the step pin
        void invert_step_polarity();

        // inverts the direction
        void invert_dir_polarity();

        // invert the direction for homing
        void invert_home_dir();

        // set the conversion from steps -> units (i.e., mm, inches, etc.)
        void set_units_per_step(float units_per_step);

        // convert steps to units
        float cvt_to_units(int32_t steps);

        // convert units to steps
        float cvt_to_steps(float units);
        
        // set the maximum allowed acceleration (in units/sec^2)
        void set_max_accel(float max_accel);

        // set the maximum and minumum speed (in units/sec)
        void set_speed_limits(float min_speed, float max_speed);

        // set the maximum and minimum travel/distance (in units)
        void set_travel_limits(float min_travel, float max_travel);

        // set the direction of travel
        void set_direction(int8_t dir);

        // set the target position in absolute coordinates (in units)
        void set_target_abs(float abs_pos);

        // set the target position in relative coordinates (in units)
        void set_target_rel(float rel_pos);

        // sets the current position to zero
        void set_zero();

        // returns the current position (in units)
        float get_position();

        // returns the (mapped) speed (in units/sec)
        float get_speed();

        // returns the steps b/w the starting position and end position
        uint32_t get_delta_steps();

        // attach a callback to be executed during homing (i.e., to read a switch/sensor)
        void set_homing_callback(int8_t (*callback)());

        // prepare the stepper for homing
        void prepare_homing();

        // check if homing is complete (1 -> complete, 0 -> incomplete, -1 -> error)
        int8_t homing_complete() __always_inline;

        // returns if the stepper is homed
        bool is_homed();

        // attach a callback to be executed during probing (i.e., to read a switch/sensor)
        void set_probing_callback(int8_t (*callback)());

        // prepare the stepper for probing
        void prepare_probing(int8_t dir);

        // check if probing is complete (1 -> complete, 0 -> incomplete, -1 -> error)
        int8_t probing_complete() __always_inline;

        // returns the axis number
        uint8_t get_axis_id() __always_inline;

        // makes sure the speed (in steps/sec) and acceleration (in steps/sec^2) are within the limits of the stepper
        // this does not need to be called manually; it is instead handled from within TMCStepperControl.start_move(...)
        void constrain_speed_accel(Stepper* master, float* start_speed, float* speed, float* accel);

        // resets internal counters; does not need to be called manually, it is called inside the stepper ISR
        void finish_move() __always_inline;

        // do a single step; does not need to be called manually, it is called inside the stepper ISR
        bool step(Stepper* master) __always_inline;

        // clear the step pin; does not need to be called manually, it is called inside the stepper ISR
        void clear_step() __always_inline;

        // returns if the target has been reached
        bool move_complete() __always_inline;

        // for sorting the steppers by distance
        static bool cmp_delta(Stepper* a, Stepper* b);

    private:
        const uint8_t dir_pin;
        const uint8_t step_pin;

        uint8_t axis;

        float min_speed;    // minimum speed (steps/sec)
        float max_speed;    // maximum speed (steps/sec)
        float max_accel;    // maximum acceleration (steps/s^2)

        int8_t dir;
        
        int32_t position;
        int32_t target_position;
        int32_t min_travel;
        int32_t max_travel;

        bool homed;
        bool invert_dir;
        bool invert_step;
        bool invert_home;

        uint32_t delta;
        int32_t delta_rem;

        int8_t probe_home_dir;
        float probe_home_scalar;

        float units_per_step;

        static uint8_t count;

        // user-defined callbacks for homing and probing
        int8_t (*homing_callback)();
        int8_t (*probing_callback)();

        // set the target position in absolute coordinates (in steps)
        void set_target_abs_steps(int32_t abs_pos);

        // set the target position in relative coordinates (in steps)
        void set_target_rel_steps(int32_t rel_pos);


        // do a single step
        bool step() __always_inline;
};

inline bool Stepper::step(Stepper* master) {
    if (this == master) return step();

    if (delta_rem >= 0) {
        delta_rem += delta - master->delta;
        return step();
    }
    delta_rem += delta;

    return true;
}

inline bool Stepper::step() {

    position += dir;
    if ((position < min_travel) || (position > max_travel)) {
        position -= dir;
        return false;
    }

    digitalWriteFast(step_pin, (!invert_step ? HIGH : LOW));
    return true;
}

inline void Stepper::clear_step() {
    digitalWriteFast(step_pin, (!invert_step ? LOW : HIGH));
}

inline bool Stepper::move_complete() {
    return position == target_position;
}

inline int8_t Stepper::probing_complete() {
    int8_t r = (*probing_callback)();
    return r == 0 ? 0 : (r > 0 ? 1 : -1);
}

inline int8_t Stepper::homing_complete() {
    int8_t r = (*homing_callback)();
    return r == 0 ? 0 : (r > 0 ? 1 : -1);
}

inline uint8_t Stepper::get_axis_id() {
    return axis;
}

inline void Stepper::finish_move() {
    delta = 0;
    probe_home_dir = 0;
    probe_home_scalar = 1;
    target_position = position;
}