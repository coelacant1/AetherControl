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
    bool homeMax = true;
    bool isHomed = false;
    bool isRelative = false;
    bool isEnabled = false;
    bool newCommand = true;

    float controlPosition = 0.0f;
    float previousTargetPosition = 0.0f;
    float previousControlPosition = 0.0f;

    float targetPosition = 0.0f;//mm
    float targetVelocity = 0.0f;//mm/s
    
    float position = 0.0f;//mm
    float velocity = 0.0f;//mm/s
    float acceleration = 0.0f;//mm/s2
    float jerk = 0.0f;//mm/s3

    float lastTravelEstimate;
    elapsedMicros sinceTravelEstimate;
    elapsedMicros sinceUpdate;

public:
    Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, float stepsPerMillimeter, float acceleration); // Relative axis
    Axis(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t endstopPin, float stepsPerMillimeter, float acceleration); // Absolute axis

    void Initialize();

    void SetAxisLimits(AxisLimits axisLimits);

    void SetHomeDirection(bool towardsMax);

    void AutoHome(); // Perform auto-homing procedure
    void Invert(bool invert);// Invert axis direction
    void Enable(); // Enables the stepper motor
    void Disable(); // Disables the stepper motor
    void SetStepsPerMillimeter(float stepsPerMM); // Configures steps per mm
    void SetAcceleration(float acceleration);

    bool ReadEndstop() const; // Reads the endstop state

    AxisLimits GetAxisLimits();
    float GetCurrentPosition(); // Returns the current position in mm
    float GetCurrentVelocity() const;
    float GetAcceleration() const;
    
    float GetStepsPerMillimeter(); // Returns steps per mm
    
    float GetPreviousTargetPosition() const;
    float GetTargetPosition() const;// 
    float GetTargetVelocity() const;// 

    void SetCurrentPosition(float position);
    void SetCurrentVelocity(float velocity);// Sets current control loop to velocity

    void SetTargetPosition(float targetPosition);// Returns constrained output
    void SetTargetVelocity(float targetVelocity);// Returns constrained output

    float GetControlPreviousPosition();// Sets current control loop to position
    void SetControlPosition(float position);// Sets current control loop to position
    void SetControlFrequency(float hertz);// Sets current control loop frequency

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
void Axis<axisCount>::SetHomeDirection(bool towardsMax){
    this->homeMax = towardsMax;
}

template<size_t axisCount>
void Axis<axisCount>::AutoHome(){
    float min, max;

    if (homeMax){
        min = axisLimits.minPosition;
        max = axisLimits.maxPosition;
    }
    else{
        min = axisLimits.maxPosition;
        max = axisLimits.minPosition;
    }

    SetCurrentPosition(min);
    SetTargetPosition(max);
    SetTargetVelocity(axisLimits.maxVelocity / 10.0f);

    while(!ReadEndstop()){
        Update();
        delay(5);
    }
    
    SetCurrentPosition(max);
    SetTargetPosition(homeMax ? max - 5.0f : max + 5.0f);
    SetTargetVelocity(axisLimits.maxVelocity / 50.0f);
    
    while(ReadEndstop() || !Mathematics::IsClose(GetCurrentPosition(), GetTargetPosition(), 0.01f)){
        Update();
        delay(5);
    }

    SetTargetPosition(max);
    SetTargetVelocity(axisLimits.maxVelocity / 50.0f);

    while(!ReadEndstop() || !Mathematics::IsClose(GetCurrentPosition(), GetTargetPosition(), 0.01f)){
        Update();
        delay(5);
    }
    
    SetCurrentPosition(max);
    SetTargetPosition(max);
}

template<size_t axisCount>
void Axis<axisCount>::Invert(bool invert){
    PulseControl<axisCount>::SetDirection(invert);
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
float Axis<axisCount>::GetStepsPerMillimeter(){ // Returns steps per mm
    return stepsPerMillimeter;
}

template<size_t axisCount>
float Axis<axisCount>::GetCurrentPosition() {
    position = PulseControl<axisCount>::GetCurrentPosition(instanceNumber) / stepsPerMillimeter;

    return position;
}

template<size_t axisCount>
void Axis<axisCount>::SetCurrentPosition(float position){
    this->position = position;

    PulseControl<axisCount>::SetCurrentPosition(instanceNumber, position * stepsPerMillimeter);
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
float Axis<axisCount>::GetPreviousTargetPosition() const {
    return previousTargetPosition;
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
void Axis<axisCount>::SetCurrentVelocity(float velocity){
    this->velocity = velocity;
}

template<size_t axisCount>
void Axis<axisCount>::SetTargetPosition(float targetPosition){
    previousTargetPosition = this->targetPosition;
    
    this->targetPosition = Mathematics::Constrain(targetPosition, axisLimits.minPosition, axisLimits.maxPosition);

    newCommand = true;
}

template<size_t axisCount>
void Axis<axisCount>::SetTargetVelocity(float targetVelocity){
    this->targetVelocity = Mathematics::Constrain(targetVelocity, axisLimits.minVelocity, axisLimits.maxVelocity);
}

template<size_t axisCount>
float Axis<axisCount>::GetControlPreviousPosition(){
    return previousControlPosition;
}

template<size_t axisCount>
void Axis<axisCount>::SetControlPosition(float position){
    previousControlPosition = controlPosition;
    controlPosition = position;

    PulseControl<axisCount>::SetTargetPosition(instanceNumber, position * stepsPerMillimeter);
}

template<size_t axisCount>
void Axis<axisCount>::SetControlFrequency(float hertz){
    long microseconds = 1000000L / Mathematics::Constrain(long(hertz), 1L, 250000L);

    PulseControl<axisCount>::SetFrequency(instanceNumber, microseconds);
}

template<size_t axisCount>
void Axis<axisCount>::Update(){
    // Time since target velocity last changed
    GetCurrentPosition();// Updates current position from step position

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

    SetControlPosition(targetPosition);

    noInterrupts();
    PulseControl<axisCount>::SetFrequency(instanceNumber, microseconds);
    interrupts();
}
