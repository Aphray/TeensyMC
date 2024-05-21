#include "StepperControl.h"


void StepperControl::addStepper(Stepper& stepper) {
    stepper.setStepperControl(this);
}

void StepperControl::sortSteppers() {

}