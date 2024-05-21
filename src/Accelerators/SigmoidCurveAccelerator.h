#pragma once

#include "AcceleratorBase.h"

class SigmoidCurveAccelerator: public AcceleratorBase {
    public:
        SigmoidCurveAccelerator();
        SigmoidCurveAccelerator(const float sigB, const float sigC);
        
    private:

        float sigA;
        const float sigB;
        const float sigC;

        float accelStopInv;

    protected:

        void precompute();

        float computeSpeed(uint32_t steps);
};