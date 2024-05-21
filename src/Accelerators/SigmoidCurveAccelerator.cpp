#include "SigmoidCurveAccelerator.h"

SigmoidCurveAccelerator::SigmoidCurveAccelerator(): sigB(10), sigC(5) {};
SigmoidCurveAccelerator::SigmoidCurveAccelerator(const float sigB, const float sigC): sigB(sigB), sigC(sigC) {};

void SigmoidCurveAccelerator::precompute() {
    accelStopInv = 1 / accelStop;
    sigA = startSpeed + (targetSpeed - startSpeed);
}

float SigmoidCurveAccelerator::computeSpeed(uint32_t steps) {
    float x = steps * accelStopInv;
    return sigA / (1 + exp(-(sigB * x - sigC)));
}