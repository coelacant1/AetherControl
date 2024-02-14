#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>
#include <functional>

#include "..\ProtoTracer\Utils\Math\Mathematics.h"
#include "..\ProtoTracer\Utils\Filter\RunningAverageFilter.h"

IntervalTimer pulseTimer;

template<size_t axisCount>
class PulseControl {
private:
    static RunningAverageFilter<10> avgFilt[axisCount];
    static volatile uint8_t dirPin[axisCount];
    static volatile uint8_t stepPin[axisCount];
    static volatile bool direction[axisCount];
    static volatile long currentPositionSteps[axisCount];
    static volatile long targetPositionSteps[axisCount];
    static volatile long minimumPositionSteps[axisCount];
    static volatile long maximumPositionSteps[axisCount];
    static volatile long frequencyCounter[axisCount];
    static volatile long frequencyTarget[axisCount];
    static volatile bool stepped[axisCount];
    static volatile bool useConstraints[axisCount];
    
public:
    static void Initialize();

    static void SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin);

    static void SetDirection(uint8_t instanceNumber, bool direction);
    
    static void SetConstraints(uint8_t instanceNumber, long minimumPositionSteps, long maximumPositionSteps);

    static void SetTargetPosition(uint8_t instanceNumber, long targetPosition);
    
    static void SetFrequency(uint8_t instanceNumber, long microseconds);

    static void AutoStepControl();

    static long GetCurrentPosition(uint8_t instanceNumber);

    static void SetCurrentPosition(uint8_t instanceNumber, long currentPositionSteps);

    static void Enable();

    static void Disable();

    static void EnableConstraints(uint8_t instanceNumber);
    static void DisableConstraints(uint8_t instanceNumber);
};


IntervalTimer stepControlTimer;

template<size_t axisCount>
RunningAverageFilter<10> PulseControl<axisCount>::avgFilt[axisCount];
template<size_t axisCount>
volatile uint8_t PulseControl<axisCount>::dirPin[axisCount];
template<size_t axisCount>
volatile uint8_t PulseControl<axisCount>::stepPin[axisCount];
template<size_t axisCount>
volatile bool PulseControl<axisCount>::direction[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::currentPositionSteps[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::targetPositionSteps[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::minimumPositionSteps[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::maximumPositionSteps[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::frequencyCounter[axisCount];
template<size_t axisCount>
volatile long PulseControl<axisCount>::frequencyTarget[axisCount];
template<size_t axisCount>
volatile bool PulseControl<axisCount>::stepped[axisCount];
template<size_t axisCount>
volatile bool PulseControl<axisCount>::useConstraints[axisCount];

template<size_t axisCount>
void PulseControl<axisCount>::Initialize(){
    for (int i = 0; i < uint8_t(axisCount); i++){
        useConstraints[i] = false;
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

                digitalWriteFast(dirPin[i], direction[i]);// Set Direction
                digitalWriteFast(stepPin[i], HIGH);// Step on

                stepped[i] = true;
            }
            else if (currentPositionSteps[i] > targetPositionSteps[i] && (currentPositionSteps[i] > minimumPositionSteps[i] || !useConstraints[i])) {
                currentPositionSteps[i]--;
                
                digitalWriteFast(dirPin[i], !direction[i]);// Set Direction
                digitalWriteFast(stepPin[i], HIGH);// Step on
                
                stepped[i] = true;
            }
        }
        
        frequencyCounter[i]++;
    }
}

template<size_t axisCount>
void PulseControl<axisCount>::Enable(){
    pulseTimer.begin(PulseControl<axisCount>::AutoStepControl, 1);//1MHz max update rate
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