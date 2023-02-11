#pragma once

#include <Arduino.h>

namespace Accelerator {

    extern float current_speed;
    extern uint32_t current_step;

    void prepare(uint32_t steps, float initial_speed, float target_speed, float accel);

    float compute_next_step_period();
}