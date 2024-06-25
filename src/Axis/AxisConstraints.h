#pragma once

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"

struct AxisConstraints {
private:
    char axisLabel = '0';
    float minPosition = 0.0f; // mm
    float maxPosition = 0.0f; // mm
    float minVelocity = 0.0f;
    float maxVelocity = 0.0f; // mm/s
    float maxAcceleration = 0.0f; // mm/s^2
    float acceleration = 0.0f;
    float stepsPerMillimeter = 0.0f;

public:
    AxisConstraints();
    
    AxisConstraints(char axisLabel, float minPosition, float maxPosition, float minVelocity, float maxVelocity, float maxAcceleration, float acceleration, float stepsPerMillimeter);

    char GetAxisLabel() const;
    float GetMinPosition() const;
    float GetMaxPosition() const;
    float GetMinVelocity() const;
    float GetMaxVelocity() const;
    float GetMaxAcceleration() const;
    float GetAcceleration() const;
    float GetStepsPerMillimeter() const;
    
    void SetAxisLabel(char axisLabel);
    void SetMinPosition(float minPosition);
    void SetMaxPosition(float maxPosition);
    void SetMinVelocity(float velocity);
    void SetMaxVelocity(float velocity);
    void SetMaxAcceleration(float acceleration);
    void SetAcceleration(float acceleration);
    void SetStepsPerMillimeter(float stepsPerMillimeter);
};
