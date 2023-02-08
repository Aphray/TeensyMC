#include "accelerator.h"
#include "serial_comm.h"
#include "configuration.h"


inline float interpf(float x0, float y0, float x1, float y1, float x) {
    return y0 + (x - x0) * (y1 - y0)/ (x1 - x0);
}

#ifdef S_CURVE_ACCELERATION
    
    const float TWO_PI_SQR = 2 * TWO_PI;
    const int MAX = sizeof(SPEED) / sizeof(float);

    float interp_period(float x) {
        float d = MAX * x;
        uint16_t i = (uint32_t) d;
        return (i >= (MAX-1)) ? PERIODS[MAX-1] : PERIODS[i] + (d - i) * (PERIODS[i+1] - PERIODS[i]);
    }

#endif

namespace Accelerator {

    uint32_t accel_stop;
    uint32_t decel_start;
    uint32_t total_steps;
    uint32_t current_step;

    float target_speed;
    float initial_speed;
    float current_speed;

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

            #else
            current_speed = interpf(0, initial_speed, accel_stop, target_speed, current_step);
            #endif
            

        } else if (current_step < decel_start) {
            // constant "cruise" period
            current_speed = target_speed;

        } else if (current_step <= total_steps) {
            // deceleration period
            #ifdef S_CURVE_ACCELERATION

            #else
            current_speed = interpf(0, initial_speed, accel_stop, target_speed, total_steps - current_step);
            #endif

        } else {
            // move complete
            current_speed = 0;
        }

        return (current_speed > 0) ? 1'000'000.0F / current_speed: -1;
    }
}