#pragma once

struct AxisLimits {
    float minPosition = 0.0f;//mm
    float maxPosition = 0.0f;//mm
    float minVelocity = 0.0f;
    float maxVelocity = 0.0f;//mm/s
    float maxAcceleration = 0.0f;//mm/s2

    AxisLimits() {}
    
    AxisLimits(float minPosition, float maxPosition, float minVelocity, float maxVelocity, float maxAcceleration) : minPosition(minPosition), maxPosition(maxPosition), minVelocity(minVelocity), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration) {}

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
};
