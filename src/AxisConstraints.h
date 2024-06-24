#pragma once

#include "../ProtoTracer/Utils/Math/Mathematics.h"

struct AxisConstraints {
private:
    char axisLabel = '0';
    float minPosition = 0.0f;//mm
    float maxPosition = 0.0f;//mm
    float minVelocity = 0.0f;
    float maxVelocity = 0.0f;//mm/s
    float maxAcceleration = 0.0f;//mm/s2
    float acceleration = 0.0f;
    float stepsPerMillimeter = 0.0f;

public:
    AxisConstraints() {}
    
    AxisConstraints(char axisLabel, float minPosition, float maxPosition, float minVelocity, float maxVelocity, float maxAcceleration, float acceleration, float stepsPerMillimeter) : axisLabel(axisLabel), minPosition(minPosition), maxPosition(maxPosition), minVelocity(minVelocity), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), stepsPerMillimeter(stepsPerMillimeter) {
        SetAcceleration(acceleration);
    }

    char GetAxisLabel() const {
        return axisLabel;
    }

    float GetMinPosition() const {
        return minPosition;
    }

    float GetMaxPosition() const {
        return maxPosition;
    }

    float GetMinVelocity() const {
        return minVelocity;
    }

    float GetMaxVelocity() const {
        return maxVelocity;
    }

    float GetMaxAcceleration() const {
        return maxAcceleration;
    }

    float GetAcceleration() const {
        return acceleration;
    }

    float GetStepsPerMillimeter() const {
        return stepsPerMillimeter;
    }
    
    void SetAxisLabel(char axisLabel){
        this->axisLabel = axisLabel;
    }

    void SetMinPosition(float minPosition){
        this->minPosition = minPosition;
    }

    void SetMaxPosition(float maxPosition){
        this->maxPosition = maxPosition;
    }

    void SetMinVelocity(float velocity){
        this->minVelocity = velocity;
    }

    void SetMaxVelocity(float velocity){
        this->maxVelocity = velocity;
    }

    void SetMaxAcceleration(float acceleration){
        this->maxAcceleration = acceleration;
    }
    
    void SetAcceleration(float acceleration){
        this->acceleration = Mathematics::Constrain(acceleration, 100.0f, maxAcceleration);

        this->acceleration = acceleration;
    }
    
    void SetStepsPerMillimeter(float stepsPerMillimeter){
        this->stepsPerMillimeter = stepsPerMillimeter;
    }
};
