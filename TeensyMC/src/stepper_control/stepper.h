#pragma once

#include <Arduino.h>
#include "../communication/enum_factory.h"


class Stepper {

    public:
        uint8_t axis;

        float max_speed;
        float min_speed;
        float max_accel;

        int32_t min_travel;
        int32_t max_travel;

        int32_t position;

        bool homed;
        
        Stepper(uint8_t step_pin, uint8_t dir_pin);

        void invert_step_polarity();
        void invert_dir_polarity();
        
        void set_max_accel(float max_accel);
        void set_speed_limits(float min_speed, float max_speed);
        void set_travel_limits(int32_t min_travel, int32_t max_travel);

        void set_direction(int8_t dir);
        void set_target_abs(int32_t abs_pos);
        void set_target_rel(int32_t rel_pos);

        void set_homing_callback(int8_t (*callback)());
        void set_probing_callback(int8_t (*callback)());

        bool step(Stepper* master) __always_inline;
        void clear_step() __always_inline;
        bool move_complete() __always_inline;

        static bool cmp_delta(Stepper* a, Stepper* b);

    private:
        const uint8_t dir_pin;
        const uint8_t step_pin;

        bool invert_step;
        bool invert_dir;

        int8_t dir;
        int32_t target_position;

        uint32_t delta;
        int32_t delta_rem;

        int8_t (*homing_callback)();
        int8_t (*probing_callback)();

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