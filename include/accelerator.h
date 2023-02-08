#pragma once

#include <Arduino.h>
#include "stepper.h"

// float get_accelerator_speed();

// uint32_t get_accelerator_step();

// void prepare_accelerator(uint32_t steps, float initial_speed, float target_speed, float accel);

// float compute_next_step_period();


namespace Accelerator {

    extern float current_speed;
    extern uint32_t current_step;

    void prepare(uint32_t steps, float initial_speed, float target_speed, float accel);

    float compute_next_step_period();
}