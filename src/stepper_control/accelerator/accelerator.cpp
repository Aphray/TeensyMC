#include "accelerator.h"


_accelerator::_accelerator() {
    current_speed = 0;
    current_step = 0;
}


void _accelerator::prepare(uint32_t steps, float initial_speed_, float target_speed_, float accel) {
    current_step = 0;
    total_steps = steps;
    initial_speed = initial_speed_;
    target_speed = target_speed_;
    accel_stop = (target_speed * target_speed - initial_speed * initial_speed) / (accel + accel);

    if (accel_stop <= (steps / 2)) {
        // trapezoid profile
        decel_start = total_steps - accel_stop;
    } else {
        // triangle profile
        accel_stop = total_steps / 2;
        decel_start = total_steps - accel_stop;
        target_speed = sqrtf(initial_speed * initial_speed + (accel + accel) * accel_stop);
    }
}

void _accelerator::reset() {
    current_speed = 0;
    current_step = 0;
}

float _accelerator::get_speed() {
    return current_speed;
}

void _accelerator::decelerate_now() {
    current_step = decel_start - 1;
}

float _accelerator::compute_next_step_period() {
    current_step++;

    if (current_step < accel_stop) {
        // acceleration period
        #ifdef SIN_CURVE_ACCELERATION
        current_speed = interp_scurve(current_step);
        #else
        current_speed = interpf(0, initial_speed, accel_stop, target_speed, current_step);
        #endif
        
    } else if (current_step < decel_start) {
        // constant "cruise" period
        current_speed = target_speed;
        return 1'000'000.0F / current_speed;

    } else if (current_step < total_steps) {
        // deceleration period
        #ifdef SIN_CURVE_ACCELERATION
        current_speed = interp_scurve(total_steps - current_step - 1);
        #else
        current_speed = interpf(0, initial_speed, accel_stop, target_speed, total_steps - current_step - 1);
        #endif

    } else {
        // move complete
        current_speed = 0;
    }

    

    return (current_speed > 0) ? 1'000'000.0F / current_speed: -1;
}

