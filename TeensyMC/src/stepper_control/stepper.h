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
        
        // set the maximum allowed acceleration (in steps/sec^2)
        void set_max_accel(float max_accel);

        // set the maximum and minumum speed (in steps/sec)
        void set_speed_limits(float min_speed, float max_speed);

        // set the maximum and minimum travel/distance (in steps)
        void set_travel_limits(int32_t min_travel, int32_t max_travel);

        // set the conversion from steps -> units (i.e., mm, inches, etc.)
        void set_units_per_step(float units_per_step);

        // returns the current position in units
        float get_position_in_units();

        // set the direction of travel
        void set_direction(int8_t dir);

        // set the target position in absolute coordinates (in steps)
        void set_target_abs(int32_t abs_pos);

        // set the target position in relative coordinates (in steps)
        void set_target_rel(int32_t rel_pos);

        // set the target position in absolute coordinates (in units)
        void set_target_abs_units(float abs_pos);

        // set the target position in relative coordinates (in units)
        void set_target_rel_units(float rel_pos);

        // attach a callback to be executed during homing (i.e., to read a switch/sensor)
        void set_homing_callback(int8_t (*callback)());

        // attach a callback to be executed during probing (i.e., to read a switch/sensor)
        void set_probing_callback(int8_t (*callback)());

        // returns the (mapped) speed (in steps/sec)
        float get_speed();

        // returns the (mapped) speed (in units/sec)
        float get_speed_in_units();

        // returns the axis number
        uint8_t get_axis_id();

        // do a single step
        bool step(Stepper* master) __always_inline;

        // clear the step pin
        void clear_step() __always_inline;

        // returns if the target has been reached
        bool move_complete() __always_inline;

        // for sorting the steppers by distance
        static bool cmp_delta(Stepper* a, Stepper* b);

    private:
        const uint8_t dir_pin;
        const uint8_t step_pin;

        uint8_t axis;

        float max_speed;
        float min_speed;
        float max_accel;

        int8_t dir;

        int32_t position;
        int32_t target_position;
        int32_t min_travel;
        int32_t max_travel;

        bool homed;
        bool invert_step;
        bool invert_dir;

        uint32_t delta;
        int32_t delta_rem;

        float units_per_step;

        int8_t (*homing_callback)();
        int8_t (*probing_callback)();

        static uint8_t count;

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