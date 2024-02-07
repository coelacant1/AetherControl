#pragma once

#include <Arduino.h>

#include "..\ProtoTracer\Utils\Math\Mathematics.h"

#include "AxisLimits.h"
#include "Constants.h"
#include "PulseControl.h"

template<size_t axisCount>
class Axis {
private:
    static uint8_t instanceCount;
    uint8_t instanceNumber = 0;
    AxisLimits axisLimits;
    uint8_t stepPin = -1;
    uint8_t dirPin = -1;
    uint8_t enablePin = -1;
    uint8_t endstopPin = -1;

    float stepsPerMillimeter = 80.0f;
    bool isHomed = false;
    bool isRelative = false;
    bool isEnabled = false;
    bool newCommand = true;

    float previousTargetPosition = 0.0f;
    float previousTargetVelocity = 0.0f;

    float targetPosition = 0.0f;//mm
    float targetVelocity = 0.0f;//mm/s
    
    float position = 0.0f;//mm
    float velocity = 0.0f;//mm/s
    float acceleration = 0.0f;//mm/s2
    float jerk = 0.0f;//mm/s3

    float lastTravelEstimate;
    elapsedMicros sinceTravelEstimate;
    elapsedMicros sinceUpdate;

    void SetDirection(bool direction);
    void Step();

public:
    Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, float stepsPerMillimeter, float acceleration); // Relative axis
    Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t endstopPin, float stepsPerMillimeter, float acceleration); // Absolute axis

    void Initialize();

    void SetAxisLimits(AxisLimits axisLimits);

    void AutoHome(); // Perform auto-homing procedure
    void Enable(); // Enables the stepper motor
    void Disable(); // Disables the stepper motor
    void SetStepsPerMillimeter(float stepsPerMM); // Configures steps per mm
    void SetAcceleration(float acceleration);

    bool ReadEndstop() const; // Reads the endstop state

    AxisLimits GetAxisLimits();
    float GetCurrentPositionMM(); // Returns the current position in mm
    float GetCurrentVelocity() const;
    float GetAcceleration() const;
    
    float GetTargetPosition() const;// 
    float GetTargetVelocity() const;// 

    void SetTargetPosition(float targetPosition);// Returns constrained output
    void SetTargetVelocity(float targetVelocity);// Returns constrained output

    float CalculateTravelTime();// Time to travel from current position, to target position, assuming cosine interpolation for position 

    void Update();//Updates the current axis to follow the command set

};

template<size_t axisCount>
uint8_t Axis<axisCount>::instanceCount = 0;

template<size_t axisCount>
Axis<axisCount>::Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, float stepsPerMillimeter, float acceleration) : stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), stepsPerMillimeter(stepsPerMillimeter), acceleration(acceleration) {
    isRelative = true;

    instanceNumber = instanceCount++;
}

template<size_t axisCount>
Axis<axisCount>::Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t endstopPin, float stepsPerMillimeter, float acceleration) : stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), endstopPin(endstopPin), stepsPerMillimeter(stepsPerMillimeter), acceleration(acceleration) {
    instanceNumber = instanceCount++;
}

template<size_t axisCount>
void Axis<axisCount>::Initialize(){
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    if (!isRelative){
        pinMode(endstopPin, INPUT_PULLUP);
    }

    PulseControl<axisCount>::SetPins(instanceNumber, stepPin, dirPin);
    
    acceleration = Mathematics::Constrain(acceleration, 100.0f, axisLimits.maxAcceleration);
}

template<size_t axisCount>
void Axis<axisCount>::SetDirection(bool direction){
    digitalWriteFast(dirPin, direction);
}

template<size_t axisCount>
void Axis<axisCount>::Step(){
    digitalWriteFast(stepPin, HIGH);// Step
    delayNanoseconds(100);
    digitalWriteFast(stepPin, LOW);
}

template<size_t axisCount>
void Axis<axisCount>::AutoHome(){

}

template<size_t axisCount>
void Axis<axisCount>::SetAxisLimits(AxisLimits axisLimits){
    this->axisLimits = axisLimits;

    PulseControl<axisCount>::SetConstraints(instanceNumber, axisLimits.minPosition * stepsPerMillimeter, axisLimits.maxPosition * stepsPerMillimeter);
}

template<size_t axisCount>
void Axis<axisCount>::Enable(){
    digitalWriteFast(enablePin, LOW);
    isEnabled = true;
}

template<size_t axisCount>
void Axis<axisCount>::SetStepsPerMillimeter(float stepsPerMillimeter){
    this->stepsPerMillimeter = stepsPerMillimeter;
}

template<size_t axisCount>
void Axis<axisCount>::SetAcceleration(float acceleration){
    acceleration = Mathematics::Constrain(acceleration, 100.0f, axisLimits.maxAcceleration);

    this->acceleration = acceleration;
}

template<size_t axisCount>
bool Axis<axisCount>::ReadEndstop() const {
    return digitalRead(endstopPin);
}

template<size_t axisCount>
void Axis<axisCount>::Disable(){
    digitalWriteFast(enablePin, HIGH);
    isEnabled = false;
}

template<size_t axisCount>
AxisLimits Axis<axisCount>::GetAxisLimits(){
    return axisLimits;
}

template<size_t axisCount>
float Axis<axisCount>::GetCurrentPositionMM() {
    noInterrupts();
    position = PulseControl<axisCount>::GetCurrentPosition(instanceNumber) / stepsPerMillimeter;
    interrupts();

    return position;
}

template<size_t axisCount>
float Axis<axisCount>::GetCurrentVelocity() const {
    return velocity;
}

template<size_t axisCount>
float Axis<axisCount>::GetAcceleration() const {
    return acceleration;
}

template<size_t axisCount>
float Axis<axisCount>::GetTargetPosition() const {
    return targetPosition;
}

template<size_t axisCount>
float Axis<axisCount>::GetTargetVelocity() const {
    return targetVelocity;
}

template<size_t axisCount>
void Axis<axisCount>::SetTargetPosition(float targetPosition){
    this->targetPosition = Mathematics::Constrain(targetPosition, axisLimits.minPosition, axisLimits.maxPosition);

    newCommand = true;

    noInterrupts();
    PulseControl<axisCount>::SetTargetPosition(instanceNumber, targetPosition * stepsPerMillimeter);
    interrupts();
}

template<size_t axisCount>
void Axis<axisCount>::SetTargetVelocity(float targetVelocity){
    this->targetVelocity = Mathematics::Constrain(targetVelocity, axisLimits.minVelocity, axisLimits.maxVelocity);
}

template<size_t axisCount>
void Axis<axisCount>::Update(){
    // Time since target velocity last changed
    GetCurrentPositionMM();// Updates current position from step position

    int direction = Mathematics::Sign(targetPosition - position);

    float dT = 0.0f;

    if (!newCommand){
        dT = float(sinceUpdate) / 1000000.0f;
    }
    else{
        newCommand = false;
        velocity = 0.0f;
    }

    sinceUpdate = 0;

    //float accelerationChange = jerk * dT;// Desired change in velocity based on acceleration
    float velocityChange = acceleration * dT * float(direction);// Desired change in velocity based on acceleration

    float remainingDistance = fabsf(targetPosition - position);// Acceleration or deceleration phase
    float decelerationDistance = (velocity * velocity) / (2.0f * acceleration);// Stopping distance


    if (position < targetPosition) {// Move forward
        if (remainingDistance > decelerationDistance && fabsf(velocity) < targetVelocity){
            velocity += velocityChange;// Accelerate
        }
        else {
            velocity -= velocityChange;// Decelerate
        }
    }
    else if (position > targetPosition){// Move backward
        if (remainingDistance > decelerationDistance && fabsf(velocity) < targetVelocity){
            velocity -= velocityChange;// Accelerate
        }
        else {
            velocity += velocityChange;// Decelerate
        }
    }

    if (velocity > axisLimits.maxVelocity) velocity = axisLimits.maxVelocity;
    if (velocity < -axisLimits.maxVelocity) velocity = -axisLimits.maxVelocity;

    if (Mathematics::IsClose(velocity, 0.0f, axisLimits.minVelocity)) velocity = 0.0f;

    if (remainingDistance < 0.001f) velocity = 0;

    long microseconds = 1000000L / Mathematics::Constrain(long(fabsf(velocity) * stepsPerMillimeter), 1L, 250000L);
    
    Serial.print(instanceNumber); Serial.print('\t');
    Serial.print(velocity); Serial.print('\t');
    Serial.print(targetVelocity); Serial.print('\t');
    Serial.print(axisLimits.maxVelocity); Serial.print('\t');
    Serial.print(position); Serial.print('\t');
    Serial.print(targetPosition); Serial.print('\t');
    Serial.print(velocityChange); Serial.print('\t');
    Serial.print(" * "); Serial.print('\t');
    Serial.print(remainingDistance); Serial.print('\t');
    Serial.print(decelerationDistance); Serial.print('\t');
    Serial.print(microseconds); Serial.print('\t');
    Serial.print(" *** "); Serial.print('\t');
    
    //Serial.println();

    noInterrupts();
    PulseControl<axisCount>::SetFrequency(instanceNumber, microseconds);
    interrupts();
}


template<size_t axisCount>
float Axis<axisCount>::CalculateTravelTime() {// Assuming S-curve time is approximately equal to linear time (integration is same result)
    GetCurrentPositionMM();

    float timeToReachTargetVelocity = targetVelocity / acceleration;
    float distanceDuringAcceleration = 0.5f * acceleration * Mathematics::Pow(timeToReachTargetVelocity, 2.0f);
    float distance = fabsf(targetPosition - position);

    // Target velocity is not reached
    if (distanceDuringAcceleration * 2 > distance) {
        float peakVelocity = Mathematics::Sqrt((distance * acceleration) / 2.0f);
        float timeToPeakVelocity = peakVelocity / acceleration;

        lastTravelEstimate = timeToPeakVelocity * 2.0f;
    }
    else {
        float distanceAtConstantVelocity = distance - (2.0f * distanceDuringAcceleration);
        float timeAtConstantVelocity = distanceAtConstantVelocity / targetVelocity;

        lastTravelEstimate = (2.0f * timeToReachTargetVelocity) + timeAtConstantVelocity;
    }

    sinceTravelEstimate = 0;

    return lastTravelEstimate;
}
