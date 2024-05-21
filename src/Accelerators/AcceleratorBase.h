#pragma once

#include <Arduino.h>

class AcceleratorBase
{
public:
    AcceleratorBase() = default;

    void reset();
    void decelerateNow();

    float getSpeed();

    bool isAccelerating();

    friend class StepperControl;

protected:
    float startSpeed;
    float targetSpeed;
    float currentSpeed;

    uint32_t currentStep;
    uint32_t accelStop;
    uint32_t decelStart;
    uint32_t totalSteps;

    void prepare(uint32_t steps, float startSpeed, float targetSpeed, float acceleration);

    float computeNextStepPeriod();

    virtual void precompute();
    virtual float computeSpeed(uint32_t steps);
};

inline float AcceleratorBase::getSpeed()
{
    return currentSpeed;
}

inline bool AcceleratorBase::isAccelerating()
{
    return (currentStep < accelStop) || (currentStep > decelStart);
}

inline void AcceleratorBase::decelerateNow()
{
    currentStep = decelStart - 1;
}

inline void AcceleratorBase::reset()
{
    currentSpeed = 0;
    currentStep = 0;
}