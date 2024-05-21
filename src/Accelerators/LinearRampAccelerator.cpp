#include "LinearRampAccelerator.h"

// inline float interpf(float x0, float y0, float x1, float y1, float x) {
//     return y0 + (x - x0) * (y1 - y0)/ (x1 - x0);
// }

void LinearRampAccelerator::precompute() {
    interpMult = (targetSpeed - startSpeed) / accelStop;
}

float LinearRampAccelerator::computeSpeed(uint32_t steps) {
    // return interpf(0, startSpeed, accelStop, targetSpeed, steps);
    return startSpeed + steps * interpMult;