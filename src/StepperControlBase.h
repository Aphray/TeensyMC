#pragma once

class StepperControlBase {
    friend class Stepper;

    private:
        virtual void sortSteppers();
};