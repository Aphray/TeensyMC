#pragma once

#include "AcceleratorBase.h"

class LinearRampAccelerator: public AcceleratorBase {

    private:

        float interpMult;
        
    protected:

        void precompute();

        float computeSpeed(uint32_t steps);
};