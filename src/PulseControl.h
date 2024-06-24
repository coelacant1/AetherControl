#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>
#include <functional>

#include "IPulseControl.h"

#include "../ProtoTracer/Utils/Math/Mathematics.h"
#include "../ProtoTracer/Utils/Filter/RunningAverageFilter.h"

IntervalTimer pulseTimer;

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
    volatile long minimumPositionSteps[axisCount];
    volatile long maximumPositionSteps[axisCount];
    volatile long frequencyCounter[axisCount];
    volatile long frequencyTarget[axisCount];
    volatile bool stepped[axisCount];
    volatile bool useConstraints[axisCount];
    
public:
    PulseControl();

    void Initialize() override;

    void SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin) override;
    void SetDirection(uint8_t instanceNumber, bool direction) override;
    void SetConstraints(uint8_t instanceNumber, long minimumPositionSteps, long maximumPositionSteps) override;
    void SetTargetPosition(uint8_t instanceNumber, long targetPosition) override;
    void SetFrequency(uint8_t instanceNumber, long microseconds) override;
    void InvertDirection(uint8_t instanceNumber, bool invert) override;

    void AutoStepControl() override;

    long GetCurrentPosition(uint8_t instanceNumber) override;
    void SetCurrentPosition(uint8_t instanceNumber, long currentPositionSteps) override;

    void Enable() override;
    void Disable() override;

    void EnableConstraints(uint8_t instanceNumber) override;
    void DisableConstraints(uint8_t instanceNumber) override;
    
    static void StaticTimerCallback();
};

template<size_t axisCount>
PulseControl<axisCount>* PulseControl<axisCount>::instances[maxInstances] = {nullptr};

template<size_t axisCount>
size_t PulseControl<axisCount>::instanceCount = 0;

template<size_t axisCount>
PulseControl<axisCount>::PulseControl(){
    if (instanceCount < maxInstances) instances[instanceCount++] = this;
}

template<size_t axisCount>
void PulseControl<axisCount>::Initialize(){
    for (int i = 0; i < uint8_t(axisCount); i++){
        useConstraints[i] = 0;
        currentPositionSteps[i] = 0;
        direction[i] = 0;
        invertDirection[i] = 0;
        currentPositionSteps[i] = 0;
        targetPositionSteps[i] = 0;
        minimumPositionSteps[i] = 0;
        maximumPositionSteps[i] = 0;
        frequencyCounter[i] = 0;
        frequencyTarget[i] = 0;
        stepped[i] = 0;
    }
}

template<size_t axisCount>
void PulseControl<axisCount>::SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin) {
    noInterrupts();
    PulseControl::stepPin[instanceNumber] = stepPin;
    PulseControl::dirPin[instanceNumber] = dirPin;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetDirection(uint8_t instanceNumber, bool direction) {
    noInterrupts();
    PulseControl::direction[instanceNumber] = direction;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetConstraints(uint8_t instanceNumber, long minimumPositionSteps, long maximumPositionSteps) {
    noInterrupts();
    PulseControl::minimumPositionSteps[instanceNumber] = minimumPositionSteps;
    PulseControl::maximumPositionSteps[instanceNumber] = maximumPositionSteps;

    avgFilt[instanceNumber] = RunningAverageFilter<10>(0.1f);
    
    useConstraints[instanceNumber] = true;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetTargetPosition(uint8_t instanceNumber, long targetPosition) {
    noInterrupts();
    targetPositionSteps[instanceNumber] = targetPosition;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetFrequency(uint8_t instanceNumber, long microseconds) {
    noInterrupts();
    frequencyTarget[instanceNumber] = microseconds;
    interrupts();
}

template<size_t axisCount>
long PulseControl<axisCount>::GetCurrentPosition(uint8_t instanceNumber){
    long temp;

    noInterrupts();
    temp = currentPositionSteps[instanceNumber];
    interrupts();

    return temp;
}

template<size_t axisCount>
void PulseControl<axisCount>::SetCurrentPosition(uint8_t instanceNumber, long steps){
    noInterrupts();
    PulseControl::currentPositionSteps[instanceNumber] = steps;//currentPositionSteps;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::InvertDirection(uint8_t instanceNumber, bool invert) {
    noInterrupts();
    PulseControl::invertDirection[instanceNumber] = invert;
    interrupts();
}


template<size_t axisCount>
void PulseControl<axisCount>::AutoStepControl(){// Controlled by interval timer
    for(uint8_t i = 0; i < axisCount; i++){
        if (frequencyCounter[i] == 9 && stepped[i]){//10 micros
            digitalWriteFast(stepPin[i], LOW);// Step off

            stepped[i] = false;
        }

        if(frequencyCounter[i] >= frequencyTarget[i]){
            frequencyCounter[i] = 0;
            
            if(currentPositionSteps[i] < targetPositionSteps[i] && (currentPositionSteps[i] < maximumPositionSteps[i] || !useConstraints[i])) {
                currentPositionSteps[i]++;

                digitalWriteFast(dirPin[i], invertDirection[i] ? !direction[i] : direction[i]);// Set Direction
                digitalWriteFast(stepPin[i], HIGH);// Step on

                stepped[i] = true;
            }
            else if (currentPositionSteps[i] > targetPositionSteps[i] && (currentPositionSteps[i] > minimumPositionSteps[i] || !useConstraints[i])) {
                currentPositionSteps[i]--;
                
                digitalWriteFast(dirPin[i], invertDirection[i] ? direction[i] : !direction[i]);// Set Direction
                digitalWriteFast(stepPin[i], HIGH);// Step on
                
                stepped[i] = true;
            }
        }
        
        frequencyCounter[i]++;
    }
}

template<size_t axisCount>
void PulseControl<axisCount>::StaticTimerCallback(){
    // Call the member function for each registered instance
    if (instances[0] != nullptr) instances[0]->AutoStepControl();
    if (instances[1] != nullptr) instances[1]->AutoStepControl();
    if (instances[2] != nullptr) instances[2]->AutoStepControl();
    if (instances[3] != nullptr) instances[3]->AutoStepControl();
    if (instances[4] != nullptr) instances[4]->AutoStepControl();
    if (instances[5] != nullptr) instances[5]->AutoStepControl();
}

template<size_t axisCount>
void PulseControl<axisCount>::Enable(){
    pulseTimer.begin(StaticTimerCallback, 1);//1MHz max update rate
}

template<size_t axisCount>
void PulseControl<axisCount>::Disable(){
    pulseTimer.end();
}

template<size_t axisCount>
void PulseControl<axisCount>::EnableConstraints(uint8_t instanceNumber){
    useConstraints[instanceNumber] = true;
}

template<size_t axisCount>
void PulseControl<axisCount>::DisableConstraints(uint8_t instanceNumber){
    useConstraints[instanceNumber] = false;
}
