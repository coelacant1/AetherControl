#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>
#include <functional>

#include "IPulseControl.h"

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"
#include "../lib/ProtoTracer/Utils/Filter/RunningAverageFilter.h"

template<size_t axisCount>
class PulseControl : public IPulseControl {
private:
    static const uint8_t maxInstances = 6;
    static PulseControl<axisCount>* instances[maxInstances];// 4 timers
    static size_t instanceCount;
    
    RunningAverageFilter<10> avgFilt[axisCount];
    volatile uint8_t dirPin[axisCount];
    volatile uint8_t stepPin[axisCount];
    volatile bool direction[axisCount];
    volatile bool invertDirection[axisCount];
    volatile long currentPositionSteps[axisCount];
    volatile long targetPositionSteps[axisCount];
    volatile long frequencyCounter[axisCount];
    volatile long frequencyTarget[axisCount];
    volatile bool stepped[axisCount];
    
    IntervalTimer pulseTimer;
    
public:
    PulseControl();

    void Initialize() override;

    void SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin) override;
    void SetDirection(uint8_t instanceNumber, bool direction) override;
    void SetTargetPosition(uint8_t instanceNumber, long targetPosition) override;
    void SetFrequency(uint8_t instanceNumber, long microseconds) override;
    void InvertDirection(uint8_t instanceNumber, bool invert) override;

    void AutoStepControl() override;

    long GetTargetPosition(uint8_t instanceNumber) override;
    long GetCurrentPosition(uint8_t instanceNumber) override;
    void SetCurrentPosition(uint8_t instanceNumber, long currentPositionSteps) override;

    void Enable() override;
    void Disable() override;
    
    static void StaticTimerCallback();
};

#include "PulseControl.tpp"