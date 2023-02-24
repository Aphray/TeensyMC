#include "stepper.h"
#include "stepper_control.h"
#include "../communication/message_agent.h"

uint8_t Stepper::count = 0;

Stepper::Stepper(uint8_t dir_pin_, uint8_t step_pin_): dir_pin(dir_pin_),step_pin(step_pin_) {

    invert_dir = false;
    invert_step = false;
    invert_home = false;
    homed = HOME_STEPPERS_FIRST ? false : true;

    axis = count++;
    
    delta = 0;
    position = 0;
    target_position = 0;
    min_overshoot = 0;
    max_overshoot = 0;

    pinMode(dir_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    set_units_per_step(1.0f);
}

void Stepper::invert_dir_polarity() {
    invert_dir = true;
}

void Stepper::invert_step_polarity() {
    invert_step = true;
}

void Stepper::invert_home_dir() {
    invert_home = true;
}

void Stepper::set_units_per_step(float units_per_step_) {
    units_per_step = units_per_step_;
}

float Stepper::cvt_to_units(int32_t steps) {
    return (float) steps * units_per_step;
}

float Stepper::cvt_to_steps(float units) {
    return units / units_per_step;
}

void Stepper::set_max_accel(float max_accel_) {
    max_accel = cvt_to_steps(max_accel_);
}

void Stepper::set_speed_limits(float min_speed_, float max_speed_) {
    min_speed = cvt_to_steps(min_speed_);
    max_speed = cvt_to_steps(max_speed_);
}

void Stepper::set_min_travel(float min_travel_) {
    min_travel = cvt_to_steps(min_travel_);
    total_travel = abs(min_travel) + abs(max_travel);
}

void Stepper::set_max_travel(float max_travel_) {
    max_travel = cvt_to_steps(max_travel_);
    total_travel = abs(min_travel) + abs(max_travel);
}

void Stepper::set_min_max_travel(float min_travel_, float max_travel_) {
    set_min_travel(min_travel_);
    set_max_travel(max_travel_);
}

float Stepper::get_position() {
    return position * units_per_step;
}

void Stepper::set_direction(int8_t dir_) {
    dir = (dir_ >= 0) ? 1 : -1;
    digitalWriteFast(dir_pin, (!invert_dir ? HIGH : LOW));
}

void Stepper::set_target_abs_steps(int32_t abs_pos) {
    set_target_rel(abs_pos - position);
}

void Stepper::set_target_rel_steps(int32_t rel_pos) {
    delta = abs(rel_pos);
    target_position = position + rel_pos;

    if (!homing && ((target_position > max_travel) || (target_position < min_travel))) {
        TMCMessageAgent.post_message(WARNING, "Target out of bounds on axis %d, limiting travel within bounds", axis);
        target_position = (target_position > max_travel) ? max_travel : min_travel;
    }

    set_direction((rel_pos >= 0) ? 1 : -1);
    TMCStepperControl.sort_steppers();
}

void Stepper::set_target_abs(float abs_pos) {
    set_target_abs_steps(cvt_to_steps(abs_pos));
}

void Stepper::set_target_rel(float rel_pos) {
    set_target_rel_steps(cvt_to_steps(rel_pos));
}

void Stepper::set_homing_callback(int8_t (*callback)()) {
    homing_callback = callback;
}

void Stepper::prepare_homing() {
    homed = false;
    homing = true;
    set_target_rel_steps((total_travel + cvt_to_steps(HOMING_OVERSHOOT)) * (invert_home ? -1 : 1));
}

bool Stepper::is_homed() {
    return homed;
}

void Stepper::set_probing_callback(int8_t (*callback)()) {
    probing_callback = callback;
}

void Stepper::prepare_probing(int8_t dir_) {
    if (dir_ >= 0) {
        min_overshoot = 0; 
        max_overshoot = cvt_to_steps(PROBING_OVERSHOOT);
        set_target_abs_steps(max_travel + max_overshoot);
    } else {
        min_overshoot = cvt_to_steps(PROBING_OVERSHOOT); 
        max_overshoot = 0;
        set_target_abs_steps(min_travel - min_overshoot);
    }
}

float Stepper::get_speed() {
    return cvt_to_units(TMCStepperControl.get_accelerator_speed() * delta / TMCStepperControl.get_master_stepper()->delta);
}

uint32_t Stepper::get_delta_steps() {
    return delta;
}

void Stepper::constrain_speed_accel(Stepper* master, float* start_speed, float* speed, float* accel) {
    float norm = delta / master->delta;

    // skip if the stepper isn't planned to move (i.e., delta = 0)
    if (norm == 0) { 
        return; 
    }

    if (max_speed < ((*speed) * norm)) {
        *speed = max_speed / norm;
    }

    if (min_speed > ((*start_speed) * norm)) {
        *start_speed = min_speed / norm;
    }

    if (max_accel < (*accel)) {
        *accel = max_accel;
    }
}

bool Stepper::cmp_delta(Stepper* a, Stepper* b) {
    return a->delta > b->delta;
}