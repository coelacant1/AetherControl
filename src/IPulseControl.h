#pragma once

#include <Arduino.h>
#include "..\ProtoTracer\Utils\Filter\RunningAverageFilter.h"

class IPulseControl {
public:
    virtual void Initialize() = 0;

    virtual void SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin) = 0;
    virtual void SetDirection(uint8_t instanceNumber, bool direction) = 0;
    virtual void SetConstraints(uint8_t instanceNumber, long minimumPositionSteps, long maximumPositionSteps) = 0;
    virtual void SetTargetPosition(uint8_t instanceNumber, long targetPosition) = 0;
    virtual void SetFrequency(uint8_t instanceNumber, long microseconds) = 0;

    virtual void AutoStepControl() = 0;

    virtual long GetCurrentPosition(uint8_t instanceNumber) = 0;
    virtual void SetCurrentPosition(uint8_t instanceNumber, long currentPositionSteps) = 0;

    virtual void Enable() = 0;
    virtual void Disable() = 0;

    virtual void EnableConstraints(uint8_t instanceNumber) = 0;
    virtual void DisableConstraints(uint8_t instanceNumber) = 0;
};