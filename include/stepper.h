#pragma once

#include <Arduino.h>
#include <algorithm>
#include <TeensyTimerTool.h>
#include "enum_factory.h"
#include "configuration.h"

using namespace TeensyTimerTool;

#define STEPPER_STATES(X)   \
    X(IDLE)                 \
    X(ACTIVE)               \
    X(HOMING)               \ 
    X(PROBING)              \
    X(FAULT)                \
    X(NEEDS_HOMING)         


enum StepperState {
    STEPPER_STATES(MAKE_ENUM)
};

class Stepper {

    public:

        int8_t direction;
        int32_t target_position;
        volatile int32_t current_position;

        float max_speed;
        float min_speed;

        float max_accel;

        uint32_t delta;
        int32_t delta_rem;

        uint8_t axis;

        static uint8_t stepper_count;

        static Stepper* master;
        static Stepper* steppers[NUM_STEPPERS + 1];

        static StepperState state;

        Stepper(uint8_t step_pin, uint8_t dir_pin, int8_t probe_pin);

        static void initialize();
        static void start_move(float speed, float accel);

        bool do_step() __always_inline;
        void clear_step() __always_inline;

        void set_direction(int8_t direction);
        void switch_direction();

        void set_step_polarity(uint8_t polarity);
        void set_probe_polarity(uint8_t polarity);
        void set_inverse_direction(bool invert);

        void set_speed_limits(float min_speed, float max_speed);
        void set_travel_limits(int32_t min_travel, int32_t max_travel);

        void set_max_accel(float max_accel);

        void set_target_abs(int32_t abs_pos);
        void set_target_rel(int32_t rel_pos);

        bool probe_triggered();
        bool endstop_triggered();

        void home();
        bool home_found();

        void probe();

        void zero();

        float map_speed(float speed);

    private:

        bool homed;
        bool probing_enabled;
        bool invert_direction;

        int32_t min_travel;
        int32_t max_travel;

        const uint8_t step_pin;
        const uint8_t dir_pin;
        const int8_t probe_pin;

        volatile uint32_t* step_set_reg;
        volatile uint32_t* step_clr_reg;
        uint32_t step_bit_mask;

        volatile uint32_t* dir_pos_reg;
        volatile uint32_t* dir_neg_reg;
        uint32_t dir_bit_mask;

        volatile uint32_t* probe_pin_reg;
        uint32_t probe_bit_mask;
        uint8_t probe_polarity;

        static OneShotTimer pulse_timer;
        static PeriodicTimer step_timer;

        static Stepper* steppers_sorted[NUM_STEPPERS + 1];

        static void step_ISR();

        static void pulse_ISR();

        static Stepper* do_bresenham_step() __always_inline;

        static bool cmp_delta(Stepper* stepper1, Stepper* stepper2);
};


inline bool Stepper::do_step() {

    current_position += direction;
    if ((current_position < min_travel) || (current_position > max_travel)) {
        current_position -= direction;
        return false;
    }

    *step_set_reg = step_bit_mask;
    return true;
}

inline void Stepper::clear_step() {
    *step_clr_reg = step_bit_mask;
}

inline void Stepper::set_direction(int8_t dir) {
    direction = (dir < 0) ? -1 : 1;
    (direction == 1) ? *dir_pos_reg = dir_bit_mask : *dir_neg_reg = dir_bit_mask;
}

inline void Stepper::switch_direction() {
    set_direction(-direction);

}

inline bool Stepper::cmp_delta(Stepper* stepper1, Stepper* stepper2) {
    return stepper1->delta > stepper2->delta;
}

inline Stepper* Stepper::do_bresenham_step() {
    //first, step the master stepper
    if (!master->do_step()) return master;
        
    Stepper** slave = steppers_sorted;
    while (*(++slave)) {
        if ((*slave)->delta_rem >= 0) {
            (*slave)->delta_rem -= master->delta;
            if (!(*slave)->do_step()) return (*slave);
        }
        (*slave)->delta_rem += (*slave)->delta;
    }

    return nullptr;
}

