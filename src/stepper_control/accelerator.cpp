#include "accelerator.h"
#include "../config.h"

#define SIG_A 10
#define SIG_B 5

float current_speed = 0;
uint32_t current_step = 0;

uint32_t accel_stop = 0;
uint32_t decel_start = 0;
uint32_t total_steps = 0;

float target_speed = 0;
float initial_speed = 0;

inline float interpf(float x0, float y0, float x1, float y1, float x) {
    return y0 + (x - x0) * (y1 - y0)/ (x1 - x0);
}

#ifdef SIN_CURVE_ACCELERATION
#include "sin_speed_table.h"

inline float interp_scurve(uint32_t step) {
    const uint32_t NUM_POINTS = sizeof(SIN_SPEED_TABLE) / sizeof(float);
    const float TWO_PI_F = (float) TWO_PI;

    float x = float(step) / accel_stop * (NUM_POINTS - 1);
    int x0 = int(x);
    
    float speed;
    if ((x0 + 1) >= NUM_POINTS) {
        speed = SIN_SPEED_TABLE[NUM_POINTS - 1];
    } else {
        speed = interpf(x0, SIN_SPEED_TABLE[x0], x0 + 1, SIN_SPEED_TABLE[x0 + 1], x);
    }
    return initial_speed + (target_speed - initial_speed) * speed / TWO_PI_F;
}

float sigmoid_speed(uint32_t step) {
    float x = float(step) / accel_stop;
    return initial_speed + (target_speed - initial_speed) / (1 + exp(-(SIG_A * x - SIG_B)));
}

#endif

bool accelerating() {
    return (current_step < accel_stop) || (current_step > decel_start);
}

float accelerator_speed() {
    return current_speed;
}

void decelerate_now() {
    current_step = decel_start - 1;
}

void prepare_accelerator(uint32_t steps, float initial_speed_, float target_speed_, float accel) {
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

// void prepare_accelerator(uint32_t steps, float initial_speed_, float target_speed_) {
//     prepare_accelerator(steps, initial_speed_, target_speed_, (target_speed_ - initial_speed_) * 1000.0 / ACCELERATION_TIME);
// }

void reset_accelerator() {
    current_speed = 0;
    current_step = 0;
}

float compute_next_step_period() {
    current_step++;

    if (current_step < accel_stop) {
        // acceleration period
        #ifdef SIN_CURVE_ACCELERATION
        // current_speed = interp_scurve(current_step);
        current_speed = sigmoid_speed(current_step);
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
        // current_speed = interp_scurve(total_steps - current_step - 1);
        current_speed = sigmoid_speed(total_steps - current_step - 1);
        #else
        current_speed = interpf(0, initial_speed, accel_stop, target_speed, total_steps - current_step - 1);
        #endif

    } else {
        // move complete
        current_speed = 0;
    }

    return (current_speed > 0) ? 1'000'000.0F / current_speed: -1;
}