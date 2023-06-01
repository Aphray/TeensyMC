#pragma once 

#include <Arduino.h>


float accelerator_speed();

void decelerate_now();

void prepare_accelerator(uint32_t steps, float initial_speed, float target_speed, float accel);

// void prepare_accelerator(uint32_t steps, float initial_speed, float target_speed);

void reset_accelerator();

float compute_next_step_period();