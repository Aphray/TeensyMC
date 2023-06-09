#include "stepper.h"
#include "stepper_control.h"
#include "../communication/serial_comm.h"

#define GUARD_ACTIVE if (!move_complete()) { return; }

using namespace TeensyMC::SerialComm;

namespace TeensyMC {
    uint8_t Stepper::count = 0;

    Stepper::Stepper(uint8_t dir_pin_, uint8_t step_pin_, uint8_t en_pin_): dir_pin(dir_pin_), step_pin(step_pin_), en_pin(en_pin_) {

        invert_home = false;
        homing_probing = false;

        axis = count++;
        
        delta = 0;
        delta_rem = -1;
        position = 0;
        target_position = 0;
        position_offset = 0;

        set_units_per_step(1.0f);
        set_steps_per_rev(200);
        set_enable_level(LOW);
        enable_homing(true);
        invert_dir_polarity(false);
        invert_step_polarity(false);
    }

    void Stepper::begin() {
        pinMode(dir_pin, OUTPUT);
        pinMode(step_pin, OUTPUT);
        pinMode(en_pin, OUTPUT);
    }

    void Stepper::reset() {
        delta = 0;
        delta_rem = -1;
        position = 0;
        target_position = 0;
        position_offset = 0;
        reset_home();
    }

    void Stepper::enable(bool state) {
        if (state) { 
            digitalWriteFast(en_pin, enable_level); 
        } else { 
            digitalWriteFast(en_pin, (enable_level == HIGH ? LOW : HIGH)); 
        }
    }

    void Stepper::invert_dir_polarity(bool invert) {
        GUARD_ACTIVE;
        if (invert) {
            dir_cw = LOW;
            dir_ccw = HIGH;
        } else {
            dir_cw = HIGH;
            dir_ccw = LOW;
        }
    }

    void Stepper::invert_step_polarity(bool invert) {
        GUARD_ACTIVE;
        if (invert) {
            step_rise_edge = LOW;
            step_fall_edge = HIGH;
        } else {
            step_rise_edge = HIGH;
            step_fall_edge = LOW;
        }
    }

    void Stepper::invert_home_dir(bool invert) {
        GUARD_ACTIVE;
        if (invert) invert_home = true;
    }

    void Stepper::set_enable_level(uint8_t level) {
        enable_level = level > 0 ? HIGH : LOW;
    }

    void Stepper::set_units_per_step(float units_per_step_) {
        GUARD_ACTIVE;
        units_per_step = units_per_step_;
    }

    void Stepper::set_steps_per_rev(uint32_t steps_per_rev_) {
        GUARD_ACTIVE;
        steps_per_rev = steps_per_rev_;
    }

    float Stepper::cvt_to_units(int32_t steps) {
        return (float) steps * units_per_step;
    }

    float Stepper::cvt_to_steps(float units) {
        return units / units_per_step;
    }

    void Stepper::set_max_accel(float max_accel_) {
        GUARD_ACTIVE;
        max_accel = cvt_to_steps(max_accel_);
    }

    float Stepper::get_max_accel() {
        return max_accel;
    }

    void Stepper::set_min_max_speed(float min_speed_, float max_speed_) {
        GUARD_ACTIVE;
        min_speed = cvt_to_steps(min_speed_);
        max_speed = cvt_to_steps(max_speed_);
    }

    void Stepper::set_min_travel(float min_travel_) {
        GUARD_ACTIVE;
        min_travel = cvt_to_steps(min_travel_);
        total_travel = abs(min_travel) + abs(max_travel);
    }

    void Stepper::set_max_travel(float max_travel_) {
        GUARD_ACTIVE;
        max_travel = cvt_to_steps(max_travel_);
        total_travel = abs(min_travel) + abs(max_travel);
    }

    void Stepper::set_min_max_travel(float min_travel_, float max_travel_) {
        GUARD_ACTIVE;
        set_min_travel(min_travel_);
        set_max_travel(max_travel_);
    }

    float Stepper::get_position() {
        return (position - position_offset) * units_per_step;
    }

    void Stepper::set_working_position(float position_) {
        GUARD_ACTIVE;
        position_offset = position - cvt_to_steps(position_);
    }

    void Stepper::reset_working_position() {
        position_offset = 0;
    }

    void Stepper::store_position(uint8_t index) {
        GUARD_ACTIVE;

        if (index >= MAX_STORED_POSITIONS) {
            SerialComm::post_message(ERROR, "Cannot store position; index %i out of bounds", index);
            return;
        }

        StoredPosition* pos = &(stored_positions[index]);
        pos->stored = true;
        pos->position = position;
    }

    void Stepper::recall_position(uint8_t index) {
        GUARD_ACTIVE;

        if (index >= MAX_STORED_POSITIONS) {
            SerialComm::post_message(ERROR, "Cannot recall position; index %i out of bounds", index);
            return;
        }

        StoredPosition* pos = &(stored_positions[index]);

        if (!pos->stored) {
            SerialComm::post_message(ERROR, "Cannot recall position; no position stored at index %i", index);
            return;
        }

        set_target_abs_steps(pos->position + position_offset);
    }

    // void Stepper::set_zero() {
    //     GUARD_ACTIVE;
        
    //     position = position_offset;
    //     target_position = position_offset;
    // }

    void Stepper::set_direction(int8_t dir_) {
        GUARD_ACTIVE;
        dir = (dir_ >= 0) ? 1 : -1;
        digitalWriteFast(dir_pin, (dir >= 0 ? dir_cw : dir_ccw));
    }

    void Stepper::set_target_abs_steps(int32_t abs_pos) {
        set_target_rel_steps(abs_pos + position_offset - position);
    }

    void Stepper::set_target_rel_steps(int32_t rel_pos) {
        GUARD_ACTIVE;
        set_direction((rel_pos >= 0) ? 1 : -1);

        steps_traveled = 0;
        target_position = position + rel_pos;

        if (!jogging && !homing_probing && ((target_position > max_travel) || (target_position < min_travel))) {
            SerialComm::post_message(WARNING, "Target out of bounds on axis %d, limiting travel within bounds", axis);
            target_position = (target_position > max_travel) ? max_travel : min_travel;
        }

        delta_rem = 0;
        delta = abs(target_position - position);

        StepperControl::internal::sort_steppers();
    }

    void Stepper::set_target_abs(float abs_pos) {
        set_target_abs_steps(cvt_to_steps(abs_pos));
    }

    void Stepper::set_target_rel(float rel_pos) {
        set_target_rel_steps(cvt_to_steps(rel_pos));
    }

    void Stepper::set_homing_callback(int8_t (*callback)()) {
        GUARD_ACTIVE;
        homing_callback = callback;
    }

    void Stepper::prepare_homing() {
        if (!homing_enabled) return;
        GUARD_ACTIVE;
        home_found = false;
        homing_probing = true;
        set_target_rel_steps((total_travel + cvt_to_steps(HOMING_OVERSHOOT)) * (invert_home ? -1 : 1));
    }

    void Stepper::enable_homing(bool enable) {
        homing_enabled = enable;
        reset_home();
    }

    bool Stepper::homed() {
        return home_found;
    }

    void Stepper::reset_home() {
        home_found = (homing_enabled ? false : true);
    }

    void Stepper::homing_complete() {
        if (!homing_enabled) return;
        home_found = true;
        position = invert_home ? min_travel : max_travel;
    }

    void Stepper::set_probing_callback(int8_t (*callback)()) {
        GUARD_ACTIVE;
        probing_callback = callback;
    }

    void Stepper::prepare_probing(int8_t dir_) {
        if (!probing_enabled) return;
        GUARD_ACTIVE;
        homing_probing = true;
        set_direction(dir_);
        set_target_rel_steps((total_travel + cvt_to_steps(PROBING_OVERSHOOT)) * dir);
    }

    void Stepper::enable_probing(bool enable) {
        probing_enabled = enable;
    }

    void Stepper::prepare_jogging(float unit_vector) {
        GUARD_ACTIVE;
        unit_vector = constrain(unit_vector, -1, 1);
        if (unit_vector == 0) {
            return;
        } else {
            set_target_rel_steps(unit_vector * steps_per_rev);
            jogging = true;
        }
    }

    float Stepper::get_speed() {
        Stepper* master = StepperControl::get_master_stepper();
        
        return (master == nullptr || master->delta == 0) ? 0 : cvt_to_units(StepperControl::get_accelerator_speed() * delta / master->delta);
    }

    float Stepper::get_max_speed() {
        return max_speed;
    }

    float Stepper::get_min_speed() {
        return min_speed;
    }

    uint32_t Stepper::get_steps_traveled() {
        return steps_traveled;
    }

    uint32_t Stepper::get_delta_steps() {
        return delta;
    }

    float Stepper::get_delta_revs() {
        return float(delta) / steps_per_rev;
    }

    void Stepper::constrain_speed_accel(Stepper* master, float* start_speed, float* speed, float* accel) {
        float norm = get_delta_revs() / master->get_delta_revs();

        // skip if the stepper isn't planned to move (i.e., delta = 0)
        if (norm == 0) { return; }

        if (max_speed < ((*speed) * norm)) { *speed = max_speed / norm; }

        if (min_speed > ((*start_speed) * norm)) { *start_speed = min_speed / norm; }

        if (max_accel < (*accel)) { *accel = max_accel; }
    }

    bool Stepper::cmp_delta(Stepper* a, Stepper* b) {
        return a->delta > b->delta;
    }

    void Stepper::init_delta_rem(Stepper* master) {
        if (this == master) return;
        delta_rem = delta - master->delta;
    }
}