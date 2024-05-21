#pragma once

#include "Stepper.h"
#include "StepperControlBase.h"
#include "Accelerators/AcceleratorBase.h"


class StepperControl: public StepperControlBase {

    public:
        StepperControl(AcceleratorBase& accelerator);

        void addStepper(Stepper& stepper);

    private:

        void sortSteppers();

};
