#include "AxisConstraints.h"

AxisConstraints::AxisConstraints() {}

AxisConstraints::AxisConstraints(char axisLabel, float minPosition, float maxPosition, float minVelocity, float maxVelocity, float maxAcceleration, float acceleration, float stepsPerMillimeter)
    : axisLabel(axisLabel), minPosition(minPosition), maxPosition(maxPosition), minVelocity(minVelocity), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), stepsPerMillimeter(stepsPerMillimeter) {
    SetAcceleration(acceleration);
}

char AxisConstraints::GetAxisLabel() const {
    return axisLabel;
}

float AxisConstraints::GetMinPosition() const {
    return minPosition;
}

float AxisConstraints::GetMaxPosition() const {
    return maxPosition;
}

float AxisConstraints::GetMinVelocity() const {
    return minVelocity;
}

float AxisConstraints::GetMaxVelocity() const {
    return maxVelocity;
}

float AxisConstraints::GetMaxAcceleration() const {
    return maxAcceleration;
}

float AxisConstraints::GetAcceleration() const {
    return acceleration;
}

float AxisConstraints::GetStepsPerMillimeter() const {
    return stepsPerMillimeter;
}
    
void AxisConstraints::SetAxisLabel(char axisLabel) {
    this->axisLabel = axisLabel;
}

void AxisConstraints::SetMinPosition(float minPosition) {
    this->minPosition = minPosition;
}

void AxisConstraints::SetMaxPosition(float maxPosition) {
    this->maxPosition = maxPosition;
}

void AxisConstraints::SetMinVelocity(float velocity) {
    this->minVelocity = velocity;
}

void AxisConstraints::SetMaxVelocity(float velocity) {
    this->maxVelocity = velocity;
}

void AxisConstraints::SetMaxAcceleration(float acceleration) {
    this->maxAcceleration = acceleration;
}
    
void AxisConstraints::SetAcceleration(float acceleration) {
    this->acceleration = Mathematics::Constrain(acceleration, 100.0f, maxAcceleration);
}

void AxisConstraints::SetStepsPerMillimeter(float stepsPerMillimeter) {
    this->stepsPerMillimeter = stepsPerMillimeter;
}
