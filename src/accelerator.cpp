#include "accelerator.h"
#include "serial_comm.h"
#include "configuration.h"


inline float interpf(float x0, float y0, float x1, float y1, float x) {
    return y0 + (x - x0) * (y1 - y0)/ (x1 - x0);
}

#ifdef S_CURVE_ACCELERATION
    #include "s_curve_speed_map.h"

    const uint32_t NUM_POINTS = sizeof(SPEED_MAP) / sizeof(float);
    const float TWO_PI_F = (float) TWO_PI;
#endif

namespace Accelerator {

    uint32_t accel_stop;
    uint32_t decel_start;
    uint32_t total_steps;
    uint32_t current_step;

    float target_speed;
    float initial_speed;
    float current_speed;

    #ifdef S_CURVE_ACCELERATION
    inline float compute_scurve(uint32_t step) {
        float x = ((float) step) / accel_stop * (NUM_POINTS - 1);
        int x0 = (int) x;
        
        float speed;
        if ((x0 + 1) >= NUM_POINTS) {
            speed = SPEED_MAP[NUM_POINTS - 1];
        } else {
            speed = interpf(x0, SPEED_MAP[x0], x0 + 1, SPEED_MAP[x0 + 1], x);
        }
        return initial_speed + (target_speed - initial_speed) * speed / TWO_PI_F;
    }
    #endif

    void prepare(uint32_t steps, float initial_speed_, float target_speed_, float accel) {
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

    float compute_next_step_period() {
        current_step++;

        if (current_step < accel_stop) {
            // acceleration period
            #ifdef S_CURVE_ACCELERATION
            current_speed = compute_scurve(current_step);
            #else
            current_speed = interpf(0, initial_speed, accel_stop, target_speed, current_step);
            #endif
            
        } else if (current_step < decel_start) {
            // constant "cruise" period
            current_speed = target_speed;

        } else if (current_step < total_steps) {
            // deceleration period
            #ifdef S_CURVE_ACCELERATION
            current_speed = compute_scurve(total_steps - current_step - 1);
            #else
            current_speed = interpf(0, initial_speed, accel_stop, target_speed, total_steps - current_step - 1);
            #endif

        } else {
            // move complete
            current_speed = 0;
        }

        return (current_speed > 0) ? 1'000'000.0F / current_speed: -1;
    }
}