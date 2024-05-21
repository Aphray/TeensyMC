#pragma once

#include <Arduino.h>

#include "StepperControlBase.h"


class Stepper{
    public:
        Stepper(uint8_t dirPin, uint8_t stepPin, uint8_t enPin);

        void begin();
        void reset();
        void enable(bool state);

        void resetHome();

        bool isHomed();
        inline bool isMoveCompleted();

        float convertStepsToUnits(int32_t steps);
        int32_t convertUnitsToSteps(float units);

        // setters
        void setEnableLevel(uint8_t level);
        void setStepLevels(uint8_t rise, uint8_t fall);
        void setDirLevels(uint8_t dirCW, uint8_t dirCCW);
        void setHomeDir(int8_t dir);
        void setUnitsPerStep(float unitsPerStep);
        void setStepsPerRev(uint32_t stepsPerRev);
        void setMaxAccel(float maxAccel);
        void setMinSpeed(float minSpeed);
        void setMaxSpeed(float maxSpeed);
        void setMinMaxSpeed(float minSpeed, float maxSpeed);
        void setMinTravel(float minTravel);
        void setMaxTravel(float maxTravel);
        void setMinMaxTravel(float minTravel, float maxTravel);
        void setDirection(int8_t dir);
        void setTargetAbs(float absPos);
        void setTargetRel(float relPos);
        void setWorkingPos(float workingos);
        void setHomingCallback(int8_t (*callback)());
        void setProbingCallback(int8_t (*callback)());

        // getters
        uint8_t getAxis();
        float getSpeed();
        float getPosition();
        float getMaxSpeed();
        float getMinSpeed();
        float getMaxAccel();
        float getTotalRevs();
        uint32_t getTotalSteps();
        uint32_t getStepsTraveled();
        inline int8_t getHomingStatus();
        inline int8_t getProbingStatus();


        friend class StepperControl;

    private:
    
        static uint8_t count;

        StepperControlBase* stepperControl;
        
        void (*sortSteppers)();

        const uint8_t dirPin;
        const uint8_t stepPin;
        const uint8_t enPin;

        float minSpeed;
        float maxSpeed;
        float maxAccel;

        int8_t enableLevel;

        uint8_t axis;
        
        int8_t dir;
        uint8_t dirCW;
        uint8_t dirCCW;

        uint8_t stepRiseEdge;
        uint8_t stepFallEdge;

        uint32_t stepsTraveled;
        int32_t targetPosition;
        int32_t positionOffset;
        volatile int32_t currentPosition;

        int32_t minTravel;
        int32_t maxTravel;
        uint32_t totalTravel;

        float unitsPerStep;
        uint32_t stepsPerRev;

        uint32_t bresenhamDelta;
        int32_t bresenhamDeltaRem;

        bool jogging;
        bool homeFound;
        bool invertHoming;
        bool homingEnabled;
        bool probingEnabled;
        bool homingOrProbing;

        int8_t (*homingCallback)();
        int8_t (*probingCallback)();

        void prepareHoming();
        void prepareProbing();
        void prepareJogging(float vector);

        inline void finishMove();

        inline bool runStep(Stepper* master);
        inline void clearStepPin();

        void setTargetAbsSteps(int32_t absPos);
        void setTargetRelSteps(int32_t relPos);

        void initDeltaRem(Stepper* master);
        static bool compareDelta(Stepper* stepper1, Stepper* stepper2);

        void setStepperControl(StepperControlBase* stepperControl);

};

inline bool Stepper::isMoveCompleted() {
    return (jogging ? false : currentPosition == targetPosition);
}

inline int8_t Stepper::getHomingStatus() {
    int8_t r = (*homingCallback)();

    if (isMoveCompleted() && r <= 0) return -1;

    return r == 0 ? 0 : (r > 0 ? 1 : -1);
}

inline int8_t Stepper::getProbingStatus() {
        int8_t r = (*probingCallback)();
    
    if (isMoveCompleted() && r <= 0) return -1;

    return r == 0 ? 0 : (r > 0 ? 1 : -1);
}
