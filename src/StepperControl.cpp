#include "StepperControl.h"

StepperControl::StepperControl(AcceleratorBase& accelerator): accelerator(accelerator) {};

void StepperControl::addStepper(Stepper& stepper) {
    stepper.setStepperControl(this);
}

void StepperControl::sortSteppers() {

}