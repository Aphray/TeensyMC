#pragma once

#include <Arduino.h>
#include "../config.h"


namespace TeensyMC {

    class Stepper {

        public:

            Stepper(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin);

            void begin();

            // reset
            void reset();

            // for setting enable pin
            void enable(bool state);

            // inverts the the HIGH/LOW signals on the step pin
            void invert_step_polarity(bool invert);

            // inverts the direction
            void invert_dir_polarity(bool invert);

            // invert the direction for homing
            void invert_home_dir(bool invert);

            // set the pin level that enables stepper
            void set_enable_level(uint8_t level);

            // set the conversion from steps -> units (i.e., mm, inches, etc.)
            void set_units_per_step(float units_per_step);

            // sets the number of steps for a full revolution (using for running steppers with different number of steps per rev; i.e., different microstep factors)
            void set_steps_per_rev(uint32_t steps_per_rev);

            // convert steps to units
            float cvt_to_units(int32_t steps);

            // convert units to steps
            float cvt_to_steps(float units);
            
            // set the maximum allowed acceleration (in units/sec^2)
            void set_max_accel(float max_accel);

            // set the maximum and minumum speed (in units/sec)
            void set_min_max_speed(float min_speed, float max_speed);

            // set the minimum travel distance (in units)
            void set_min_travel(float min_travel);

            // set the maximum travel distance (in units)
            void set_max_travel(float max_travel);

            // set the maximum and minimum travel distance (in units)
            void set_min_max_travel(float min_travel, float max_travel);

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

            // stores the current position with the index
            void store_position(uint8_t index);

            // recall to stored position at index
            void recall_position(uint8_t index);

            // returns the (mapped) speed (in units/sec)
            float get_speed();

            // returns the maximum specified speed (units/sec)
            float get_max_speed();

            // returns the minimum specified speed (units/sec)
            float get_min_speed();

            // returns the number of steps traveled from the starting position
            uint32_t get_steps_traveled();

            // returns the steps b/w the starting position and end position
            uint32_t get_delta_steps();

            // get delta in revolutions (normalized among all steppers)
            float get_delta_revs();

            // attach a callback to be executed during homing (i.e., to read a switch/sensor)
            void set_homing_callback(int8_t (*callback)());

            // prepare the stepper for homing
            void prepare_homing();

            // check if homing is complete (1 -> complete, 0 -> incomplete, -1 -> error)
            inline int8_t homing_status();

            // set if the axis requires homing
            void enable_homing(bool enable);

            // called on homing complete to update the position
            void homing_complete();

            // returns if the stepper has been homed
            bool homed();

            // for resetting the home status
            void reset_home();

            // attach a callback to be executed during probing (i.e., to read a switch/sensor)
            void set_probing_callback(int8_t (*callback)());

            // prepare the stepper for probing
            void prepare_probing(int8_t dir);

            // check if probing is complete (1 -> complete, 0 -> incomplete, -1 -> error)
            inline int8_t probing_status();

            // set if the axis has probing capabilities
            void enable_probing(bool enable);

            // puts the stepper in jogging mode
            void prepare_jogging(float unit_vector);

            // returns the axis number
            inline uint8_t get_axis_id();

            // makes sure the speed (in steps/sec) and acceleration (in steps/sec^2) are within the limits of the stepper
            // this does not need to be called manually; it is instead handled from within TMCStepperControl.start_move(...)
            void constrain_speed_accel(Stepper* master, float* start_speed, float* speed, float* accel);

            // resets internal counters; does not need to be called manually, it is called inside the stepper ISR
            inline void finish_move();

            // do a single step; does not need to be called manually, it is called inside the stepper ISR
            inline bool step(Stepper* master);

            // clear the step pin; does not need to be called manually, it is called inside the stepper ISR
            inline void clear_step();

            // returns if the target has been reached
            inline bool move_complete();

            // for sorting the steppers by distance
            static bool cmp_delta(Stepper* a, Stepper* b);

            void init_delta_rem(Stepper* master);

        private:

            struct StoredPosition{
                bool stored;
                int32_t position;
            };

            const uint8_t dir_pin;
            const uint8_t step_pin;
            const uint8_t en_pin;

            int8_t enable_level;

            uint8_t axis;

            float min_speed;    // minimum speed (steps/sec)
            float max_speed;    // maximum speed (steps/sec)
            float max_accel;    // maximum acceleration (steps/s^2)

            int8_t dir;         // current direction (-1 or 1)
            uint8_t dir_cw;     // pin state for clockwise rotation
            uint8_t dir_ccw;    // pin state for counter-clockwise rotation

            uint8_t step_rise_edge;     // step rising edge
            uint8_t step_fall_edge;     // step falling edge

            uint32_t steps_traveled;
            int32_t target_position;
            volatile int32_t position;

            StoredPosition stored_positions[MAX_STORED_POSITIONS];

            int32_t min_travel;
            int32_t max_travel;
            uint32_t total_travel;

            bool jogging;
            bool home_found;
            bool invert_home;
            bool homing_enabled;
            bool probing_enabled;
            bool homing_probing;

            uint32_t delta;
            int32_t delta_rem;

            int8_t probe_home_dir;

            float min_overshoot;
            float max_overshoot;

            float units_per_step;
            uint32_t steps_per_rev;

            static uint8_t count;

            // user-defined callback for homing
            int8_t (*homing_callback)();
            // user-defined callback for probing
            int8_t (*probing_callback)();

            // set the target position in absolute coordinates (in steps)
            void set_target_abs_steps(int32_t abs_pos);

            // set the target position in relative coordinates (in steps)
            void set_target_rel_steps(int32_t rel_pos);

            // do a single step
            __always_inline bool step();
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
        steps_traveled++;

        if (!homing_probing && ((position < min_travel) || (position > max_travel))) {
            position -= dir;
            steps_traveled--;
            return false;
        }

        digitalWriteFast(step_pin, step_rise_edge);
        return true;
    }

    inline void Stepper::clear_step() {
        digitalWriteFast(step_pin, step_fall_edge);
    }

    inline bool Stepper::move_complete() {
        return (jogging ? false : position == target_position);
    }

    inline int8_t Stepper::probing_status() {
        int8_t r = (*probing_callback)();
        if (move_complete() && r <= 0) { return -1; }
        return r == 0 ? 0 : (r > 0 ? 1 : -1);
    }

    inline int8_t Stepper::homing_status() {
        int8_t r = (*homing_callback)();
        if (move_complete() && r <= 0) { return -1; }
        return r == 0 ? 0 : (r > 0 ? 1 : -1);
    }

    inline uint8_t Stepper::get_axis_id() {
        return axis;
    }

    inline void Stepper::finish_move() {
        delta = 0;
        delta_rem = -1;
        jogging = false;
        homing_probing = false;
        target_position = position;
    }

}