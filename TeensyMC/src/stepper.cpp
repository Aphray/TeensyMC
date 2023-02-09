#include "stepper.h"
#include "serial_comm.h"
#include "accelerator/accelerator.h"

OneShotTimer Stepper::pulse_timer;
PeriodicTimer Stepper::step_timer;

uint8_t Stepper::stepper_count = 0;

Stepper* Stepper::master = nullptr;
Stepper* Stepper::steppers[NUM_STEPPERS + 1];
Stepper* Stepper::steppers_sorted[NUM_STEPPERS + 1];

StepperState Stepper::state = IDLE;

Stepper::Stepper(uint8_t step_pin_, uint8_t dir_pin_, int8_t probe_pin_): 
    step_pin(step_pin_), 
    dir_pin(dir_pin_), 
    probe_pin(probe_pin_) {

    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(probe_pin, INPUT);

    set_step_polarity(HIGH);
    set_probe_polarity(HIGH);
    set_inverse_direction(false);

    if (step_pin >= 0) {
        probe_pin_reg = portInputRegister(probe_pin);
        probe_bit_mask = digitalPinToBitMask(probe_pin);
    }

    delta = 1;
    current_position = 0;
    probing_enabled = false;
    master = this;

    axis = stepper_count;
    steppers[stepper_count] = this;
    steppers_sorted[stepper_count++] = this;
}

void Stepper::initialize() {
    pulse_timer.begin(pulse_ISR);
    step_timer.begin(step_ISR, 1000, false);
}

void Stepper::start_move(float speed, float accel) {
    if (master->delta == 0) return;

    float start_speed = master->min_speed;

    if (master->max_accel < accel) accel = master->max_accel;
    if (master->max_speed < speed) speed = master->max_speed;

    for (uint8_t n = 1; n < NUM_STEPPERS; n++) {
        steppers_sorted[n]->delta_rem = 2 * steppers_sorted[n]->delta - master->delta;
        if (steppers_sorted[n]->max_accel < accel) accel = steppers_sorted[n]->max_accel;
        if (steppers_sorted[n]->min_speed > start_speed) start_speed = steppers_sorted[n]->min_speed;

        float norm = steppers_sorted[n]->delta / master->delta;
        if (steppers_sorted[n]->max_speed < (norm * speed)) speed = steppers_sorted[n]->max_speed / norm;
    }

    if (state != HOMING || state != PROBING) {
        state = ACTIVE;
        SerialCommunication::post_message(INFO, "Move started");
    }

    Accelerator::prepare(master->delta, start_speed, speed, accel);
    step_timer.setPeriod(1);
    step_timer.start();
}


void Stepper::set_speed_limits(float min_speed_, float max_speed_) {
    min_speed = min_speed_;
    max_speed = max_speed_;
}

void Stepper::set_travel_limits(int32_t min_travel_, int32_t max_travel_) {
    min_travel = min_travel_;
    max_travel = max_travel_;
}   

void Stepper::set_max_accel(float accel) {
    max_accel = accel;
}

void Stepper::set_target_rel(int32_t rel_pos) {
    delta = abs(rel_pos);
    target_position = current_position + rel_pos;
    if (target_position > max_travel || target_position < min_travel) {
        SerialCommunication::post_message(WARNING, "Target of bounds on axis %d, limiting travel within bounds", axis);
        target_position = (target_position > max_travel) ? max_travel : min_travel;
    }

    set_direction((rel_pos >= 0) ? 1 : -1);

    std::sort(steppers_sorted, steppers_sorted + stepper_count, cmp_delta);
    master = steppers_sorted[0];
}

void Stepper::set_target_abs(int32_t abs_pos) {
    set_target_rel(abs_pos - current_position);
}

void Stepper::set_step_polarity(uint8_t polarity) {
    step_bit_mask = digitalPinToBitMask(step_pin);

    if (polarity == LOW) {
        step_set_reg = portClearRegister(step_pin);
        step_clr_reg = portSetRegister(step_pin);
    } else {
        step_set_reg = portSetRegister(step_pin);
        step_clr_reg = portClearRegister(step_pin);
    }
}

void Stepper::set_probe_polarity(uint8_t polarity) {
    probe_polarity = polarity;
}

void Stepper::set_inverse_direction(bool invert) {
    dir_bit_mask = digitalPinToBitMask(dir_pin);

    if (invert) {
        dir_pos_reg = portClearRegister(dir_pin);
        dir_neg_reg = portSetRegister(dir_pin);
    } else {
        dir_pos_reg = portSetRegister(dir_pin);
        dir_neg_reg = portClearRegister(dir_pin);
    }

    invert_direction = invert;
}

bool Stepper::probe_triggered() {
    if (probe_pin > 0) {
        uint8_t polarity = (*probe_pin_reg & probe_bit_mask) ? HIGH : LOW;
        if (polarity == probe_polarity) return true;
    }

    return false;
}

bool Stepper::endstop_triggered() {
    return false;
}

void Stepper::home() {

}

bool Stepper::home_found() {
    return true;
}

void Stepper::probe() {
    if (probe_pin < 0) {
        SerialCommunication::post_message(ERROR, "Probing not enabled on axis %d", axis);
        return;
    }
}

void Stepper::zero() {
    current_position = 0;
}

float Stepper::map_speed(float speed) {
    return (master->delta > 0) ? (speed * delta) / master->delta : 0;
}

void Stepper::step_ISR() {

    switch(Stepper::state) {
        case NEEDS_HOMING:
        case FAULT:
        case IDLE:
            step_timer.stop();
            break;

        case PROBING:
            if (master->probe_triggered()) {
                state = IDLE;
                step_timer.stop();
                Accelerator::current_speed = 0;
                SerialCommunication::queue_message(INFO, "Probe completed on axis %d", master->axis);
                SerialCommunication::queue_realtime_status();
                break;
            }
            goto case_ACTIVE;

        case HOMING:
            if (master->endstop_triggered()) {
                // handle home position
                
            }
            goto case_ACTIVE;
        
        case ACTIVE:
        case_ACTIVE:
        {
            Stepper* fault = do_bresenham_step();

            if (fault) {
                state = FAULT;
                step_timer.stop();
                SerialCommunication::queue_message(CRITICAL, "Fault: software limit on axis %d", fault->axis);

            } else if (master->current_position == master->target_position) {
                state = IDLE;
                step_timer.stop();
                Accelerator::current_speed = 0;
                SerialCommunication::queue_message(INFO, "Move complete");
                SerialCommunication::queue_realtime_status();

            } else {
                step_timer.setPeriod(Accelerator::compute_next_step_period());
            }

            pulse_timer.trigger(PULSE_WIDTH_MICROS);
            break;
        }
    }
}

void Stepper::pulse_ISR() {

    Stepper** stepper = steppers_sorted;
    while (*stepper) {
        (*stepper)->clear_step();
        stepper++;
    }
}