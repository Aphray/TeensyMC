#pragma once

#include "Stepper.h"
#include "StepperControlBase.h"


class StepperControl: public StepperControlBase {

    public:

        void addStepper(Stepper& stepper);

    private:

        void sortSteppers();

};
