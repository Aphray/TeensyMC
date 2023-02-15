#include "stepper.h"
#include "stepper_control.h"
#include "../communication/message_agent.h"

uint8_t Stepper::count = 0;

Stepper::Stepper(uint8_t dir_pin_, uint8_t step_pin_): dir_pin(dir_pin_),step_pin(step_pin_) {
    pinMode(dir_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);

    homed = true;
    invert_dir = false;
    invert_step = false;

    axis = count++;
    
    position = 0;
    target_position = 0;

    set_units_per_step(1.0f);
}

void Stepper::invert_dir_polarity() {
    invert_dir = true;
}

void Stepper::invert_step_polarity() {
    invert_step = true;
}

void Stepper::set_max_accel(float max_accel_) {
    max_accel = max_accel_;
}

void Stepper::set_speed_limits(float min_speed_, float max_speed_) {
    min_speed = min_speed_;
    max_speed = max_speed_;
}

void Stepper::set_travel_limits(int32_t min_travel_, int32_t max_travel_) {
    min_travel = min_travel_;
    max_travel = max_travel_;
}

void Stepper::set_units_per_step(float units_per_step_) {
    units_per_step = units_per_step_;
}

float Stepper::get_position_in_units() {
    return position * units_per_step;
}

void Stepper::set_direction(int8_t dir_) {
    dir = (dir_ >= 0) ? 1 : -1;
    digitalWriteFast(dir_pin, (!invert_dir ? HIGH : LOW));
}

void Stepper::set_target_abs(int32_t abs_pos) {
    set_target_rel(abs_pos - position);
}

void Stepper::set_target_rel(int32_t rel_pos) {
    delta = abs(rel_pos);
    target_position = position + rel_pos;

    if ((target_position > max_travel) || (target_position < min_travel)) {
        TMCMessageAgent.post_message(WARNING, 
            "Target out of bounds on axis %d, limiting travel within bounds", axis);

        target_position = (target_position > max_travel) ? max_travel : min_travel;
    }

    set_direction((rel_pos >= 0) ? 1 : -1);
    TMCStepperControl.sort_steppers();
}

void Stepper::set_target_abs_units(float abs_pos) {
    set_target_abs((int32_t) (abs_pos / units_per_step));
}

void Stepper::set_target_rel_units(float rel_pos) {
    set_target_rel((int32_t) (rel_pos / units_per_step));
}

void Stepper::set_homing_callback(int8_t (*callback)()) {
    homed = false;
    homing_callback = callback;
}

void Stepper::set_probing_callback(int8_t (*callback)()) {
    probing_callback = callback;
}

float Stepper::get_speed() {
    return (TMCStepperControl.get_accelerator_speed() * delta / TMCStepperControl.get_master_stepper()->delta);
}

float Stepper::get_speed_in_units() {
    return get_speed() * units_per_step;
}

uint8_t Stepper::get_axis_id() {
    return axis;
}

void Stepper::prepare(Stepper* master, float* start_speed, float* speed, float* accel) {
    float norm = delta / master->delta;

    
}

bool Stepper::cmp_delta(Stepper* a, Stepper* b) {
    return a->delta > b->delta;
}