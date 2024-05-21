#include "AcceleratorBase.h"

void AcceleratorBase::prepare(uint32_t steps, float startSpeed_, float targetSpeed_, float accel) {
    currentStep = 0;
    totalSteps = steps;
    startSpeed = startSpeed_;
    targetSpeed = targetSpeed_;
    accelStop = (targetSpeed * targetSpeed - startSpeed * startSpeed) / (accel + accel);

    if (accelStop <= (steps / 2)) {
        decelStart = totalSteps - accelStop;
    } else {
        accelStop = totalSteps / 2;
        decelStart = totalSteps - accelStop;
        targetSpeed = sqrtf(startSpeed * startSpeed + (accel + accel) * accelStop);
    }

    precompute();
}