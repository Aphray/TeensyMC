#pragma once

#include <Arduino.h>
#include "../../TMC_default_config.h"

float interpf(float x0, float y0, float x1, float y1, float x) __always_inline;

class _accelerator {

    public:
        float current_speed;
        uint32_t current_step;

        _accelerator();

        void prepare(uint32_t steps, float initial_speed, float target_speed, float accel);

        float compute_next_step_period();

    private:

        uint32_t accel_stop;
        uint32_t decel_start;
        uint32_t total_steps;

        float target_speed;
        float initial_speed;

        #ifdef SIN_CURVE_ACCELERATION
        float interp_scurve(uint32_t step);
        #endif
};

inline float interpf(float x0, float y0, float x1, float y1, float x) {
    return y0 + (x - x0) * (y1 - y0)/ (x1 - x0);
}

#if(SIN_CURVE_ACCELERATION)
#include "sin_speed_table.h"

inline float _accelerator::interp_scurve(uint32_t step) {
    const uint32_t NUM_POINTS = sizeof(SIN_SPEED_TABLE) / sizeof(float);
    const float TWO_PI_F = (float) TWO_PI;

    float x = ((float) step) / accel_stop * (NUM_POINTS - 1);
    int x0 = (int) x;
    
    float speed;
    if ((x0 + 1) >= NUM_POINTS) {
        speed = SIN_SPEED_TABLE[NUM_POINTS - 1];
    } else {
        speed = interpf(x0, SIN_SPEED_TABLE[x0], x0 + 1, SIN_SPEED_TABLE[x0 + 1], x);
    }
    return initial_speed + (target_speed - initial_speed) * speed / TWO_PI_F;
}
#endif