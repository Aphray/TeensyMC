#include "Stepper.h"

#define GUARD_ACTIVE if (!isMoveCompleted()) return

uint8_t Stepper::count = 0;

Stepper::Stepper(uint8_t dirPin, uint8_t stepPin, uint8_t enPin): 
dirPin(dirPin), stepPin(stepPin), enPin(enPin) {

    invertHoming = false;
    homingOrProbing = false;

    axis = count++;

    reset();

    setUnitsPerStep(1.0f);
    setStepsPerRev(200);
    setEnableLevel(LOW);
    setDirLevels(HIGH, LOW);
    setStepLevels(LOW, HIGH);
}

void Stepper::begin() {
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(enPin, OUTPUT);
}

void Stepper::reset() {
    positionOffset = 0;
    targetPosition = 0;
    currentPosition = 0;

    bresenhamDelta = 0;
    bresenhamDeltaRem = -1;
    
    resetHome();
}

void Stepper::enable(bool state) {
    if (state) digitalWriteFast(enPin, enableLevel);
    else digitalWriteFast(enPin, (enableLevel == HIGH ? LOW : HIGH));
}

void Stepper::resetHome() {
    homeFound = (homingEnabled ? false : true);
}

bool Stepper::isHomed() {
    return homeFound;
}

float Stepper::convertStepsToUnits(int32_t steps) {
    return (float) steps * unitsPerStep;
}

int32_t Stepper::convertUnitsToSteps(float units) {
    return (int32_t) (units / unitsPerStep);
}

void Stepper::setEnableLevel(uint8_t level) {
    enableLevel = level;
}

void Stepper::setStepLevels(uint8_t riseEdge, uint8_t fallEdge) {
    GUARD_ACTIVE;
    stepRiseEdge = riseEdge;
    stepFallEdge = fallEdge;
}

void Stepper::setDirLevels(uint8_t dirCW, uint8_t dirCCW) {
    GUARD_ACTIVE;
    this->dirCW = dirCW;
    this->dirCCW = dirCCW;
}

void Stepper::setHomeDir(int8_t dir) {
    GUARD_ACTIVE;
    invertHoming = (dir < 0) ? true : false;
}

void Stepper::setUnitsPerStep(float unitsPerStep) {
    GUARD_ACTIVE;
    this->unitsPerStep = unitsPerStep;
}

void Stepper::setStepsPerRev(uint32_t stepsPerRev) {
    GUARD_ACTIVE;
    this->stepsPerRev = stepsPerRev;
}

void Stepper::setMaxAccel(float maxAccel) {
    GUARD_ACTIVE;
    this->maxAccel = convertUnitsToSteps(maxAccel);
}

void Stepper::setMinSpeed(float minSpeed) {
    GUARD_ACTIVE;
    this->minSpeed = convertUnitsToSteps(minSpeed);
}

void Stepper::setMaxSpeed(float maxSpeed) {
    GUARD_ACTIVE;
    this->maxSpeed = convertUnitsToSteps(maxSpeed);
}

void Stepper::setMinMaxSpeed(float minSpeed, float maxSpeed) {
    setMinSpeed(minSpeed);
    setMaxSpeed(maxSpeed);
}

void Stepper::setMinTravel(float minTravel) {
    GUARD_ACTIVE;
    this->minTravel = convertUnitsToSteps(minTravel);
}

void Stepper::setMaxTravel(float maxTravel) {
    GUARD_ACTIVE;
    this->maxTravel = convertUnitsToSteps(maxTravel);
}

void Stepper::setMinMaxTravel(float minTravel, float maxTravel) {
    setMinTravel(minTravel);
    setMaxTravel(maxTravel);
}

void Stepper::setDirection(int8_t dir) {
    GUARD_ACTIVE;
    this->dir = (dir >= 0) ? 1 : -1;
    digitalWriteFast(dirPin, (this->dir == 1) ? dirCW : dirCCW);
}

void Stepper::setTargetAbs(float absPos) {
    setTargetAbsSteps(convertUnitsToSteps(absPos));
}

void Stepper::setTargetRel(float relPos) {
    setTargetRelSteps(convertUnitsToSteps(relPos));
}

void Stepper::setHomingCallback(int8_t (*callback)()) {
    GUARD_ACTIVE;
    homingCallback = callback;
}

void Stepper::setProbingCallback(int8_t (*callback)()) {
    GUARD_ACTIVE;
    probingCallback = callback;
}



void Stepper::setTargetAbsSteps(int32_t absPos) {
    setTargetRelSteps(absPos + positionOffset - currentPosition);
}

void Stepper::setTargetRelSteps(int32_t relPos) {
    GUARD_ACTIVE;
    setDirection(relPos);

    stepsTraveled = 0;
    targetPosition = currentPosition + relPos;

    bresenhamDeltaRem = 0;
    bresenhamDelta = abs(targetPosition - currentPosition);

    stepperControl->sortSteppers();
}


void Stepper::setStepperControl(StepperControlBase* stepperControl) {
    this->stepperControl = stepperControl;
}