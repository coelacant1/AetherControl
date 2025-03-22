#include "Axis.h"

uint8_t Axis::instanceCount = 0;

Axis::Axis(IPulseControl* pulseControl, AxisConstraints* axisConstraints, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin) : pulseControl(pulseControl), axisConstraints(axisConstraints), stepPin(stepPin), dirPin(dirPin), enablePin(enablePin) {
    isRelative = true;

    instanceNumber = instanceCount++;
}

Axis::Axis(IPulseControl* pulseControl, AxisConstraints* axisConstraints, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t endstopPin) : pulseControl(pulseControl), axisConstraints(axisConstraints), stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), endstopPin(endstopPin) {
    instanceNumber = instanceCount++;
}

void Axis::Initialize(){
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    if (!isRelative){
        pinMode(endstopPin, INPUT_PULLUP);
    }

    pulseControl->SetPins(instanceNumber, stepPin, dirPin);
}

void Axis::SetHomeDirection(bool towardsMax){
    this->homeMax = towardsMax;
}

void Axis::ResetRelative(){
    if (isRelative){
        SetCurrentPosition(0.0f);
        SetTargetPosition(0.0f);
    }
    else{
        SerialHandler::SendMessage("Axis is not relative, cannot be reset");
    }
}

void Axis::AutoHome(){// Cartesian only
    if(!isRelative){
        float min, max;
        
        // Temporarily assumes axis is relative to determine new homing location
        isHoming = true;

        if (homeMax){
            min = axisConstraints->GetMinPosition();
            max = axisConstraints->GetMaxPosition();
        }
        else{
            min = axisConstraints->GetMaxPosition();
            max = axisConstraints->GetMinPosition();
        }

        SetCurrentPosition(min);
        SetTargetPosition(homeMax ? max + GlobalVariables::homingDistanceOffset : max - GlobalVariables::homingDistanceOffset);// Move 5mm past extreme limit
        SetTargetVelocity(GlobalVariables::homingSpeedPrimary);

        while(!ReadEndstop()){
            Update();
            delay(5);
        }
        
        SetCurrentPosition(max);
        SetTargetPosition(homeMax ? max - GlobalVariables::homingDistanceBackoff : max + GlobalVariables::homingDistanceBackoff);// Move 5mm below endstop distance
        SetTargetVelocity(GlobalVariables::homingSpeedSecondary);

        while(ReadEndstop() || !Mathematics::IsClose(GetCurrentPosition(), GetTargetPosition(), 0.01f)){
            Update();
            delay(5);
        }

        SetTargetPosition(homeMax ? max + GlobalVariables::homingDistanceBackoff : max - GlobalVariables::homingDistanceBackoff);// Move 5mm past extreme limit
        SetTargetVelocity(GlobalVariables::homingSpeedSecondary);

        while(!ReadEndstop()){
            Update();
            delay(5);
        }
        
        SetCurrentPosition(max);
        SetTargetPosition(homeMax ? max - GlobalVariables::homingDistanceBackoff : max + GlobalVariables::homingDistanceBackoff);// Move 5mm below endstop distance
        SetTargetVelocity(GlobalVariables::homingSpeedSecondary);


        while(ReadEndstop() || !Mathematics::IsClose(GetCurrentPosition(), GetTargetPosition(), 0.01f)){
            Update();
            delay(5);
        }

        // Sets axis back to absolute positioning
        isHoming = false;
    }
    else{
        SerialHandler::SendMessage("Axis is relative, cannot be autohomed");
    }
}

void Axis::InvertDirection(bool invert){
    invertDirection = invert;

    pulseControl->InvertDirection(instanceNumber, invert);
}

void Axis::Enable(){
    digitalWrite(enablePin, LOW);
    isEnabled = true;
}


bool Axis::IsRelative() const {
    return isRelative;
}

bool Axis::ReadEndstop() const {
    return digitalRead(endstopPin);
}

AxisConstraints* Axis::GetAxisConstraints(){
    return axisConstraints;
}

void Axis::Disable(){
    digitalWrite(enablePin, HIGH);
    isEnabled = false;
}

float Axis::GetCurrentPosition() {
    position = pulseControl->GetCurrentPosition(instanceNumber) / axisConstraints->GetStepsPerMillimeter();

    return position;
}

void Axis::SetCurrentPosition(float position){
    this->position = position;

    pulseControl->SetCurrentPosition(instanceNumber, this->position * axisConstraints->GetStepsPerMillimeter());
}

float Axis::GetCurrentVelocity() const {
    return velocity;
}

float Axis::GetPreviousTargetPosition() const {
    return previousTargetPosition;
}

float Axis::GetTargetPosition() const {
    return targetPosition;
}

float Axis::GetTargetVelocity() const {
    return targetVelocity;
}

void Axis::SetCurrentVelocity(float velocity){
    this->velocity = velocity;

    long microseconds = 1000000L / Mathematics::Constrain(long(fabsf(velocity) * axisConstraints->GetStepsPerMillimeter()), 1L, 250000L);
    
    pulseControl->SetFrequency(instanceNumber, microseconds);
}

void Axis::SetTargetPosition(float targetPosition){
    if(isRelative){
        previousTargetPosition = this->targetPosition;
        this->targetPosition = targetPosition;
    }
    else{
        previousTargetPosition = this->targetPosition;
        
        if(!isHoming) this->targetPosition = Mathematics::Constrain(targetPosition, axisConstraints->GetMinPosition(), axisConstraints->GetMaxPosition());
        else {// Resets to relative position, always assuming it starts at zero
            this->targetPosition = targetPosition;
        }
    }
    
    newCommand = true;

    pulseControl->SetTargetPosition(instanceNumber, long(this->targetPosition * axisConstraints->GetStepsPerMillimeter()));
}

void Axis::SetTargetVelocity(float targetVelocity){
    this->targetVelocity = Mathematics::Constrain(targetVelocity, axisConstraints->GetMinVelocity(), axisConstraints->GetMaxVelocity());
}


long Axis::GetAbsoluteCurrentPosition(){
    return pulseControl->GetCurrentPosition(instanceNumber);
}

long Axis::GetAbsoluteTargetPosition(){
    return pulseControl->GetTargetPosition(instanceNumber);
}

void Axis::Update(){
    // Time since target velocity last changed
    GetCurrentPosition();// Updates current position from absolute step position

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

    float velocityChange = axisConstraints->GetAcceleration() * dT * float(direction);// Desired change in velocity based on acceleration
    float remainingDistance = fabsf(targetPosition - position);// Acceleration or deceleration phase
    float decelerationDistance = (velocity * velocity) / (2.0f * axisConstraints->GetAcceleration());// Stopping distance

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

    if (velocity > axisConstraints->GetMaxVelocity()) velocity = axisConstraints->GetMaxVelocity();
    if (velocity < -axisConstraints->GetMaxVelocity()) velocity = -axisConstraints->GetMaxVelocity();

    //if (Mathematics::IsClose(velocity, axisConstraints->GetMinVelocity(), 0.005f)) velocity = 0.0f;

    if (remainingDistance < 0.005f) velocity = 0;

    SetCurrentVelocity(velocity);
}
