#pragma once

struct AxisLimits {
    float minPosition = 0.0f;//mm
    float maxPosition = 0.0f;//mm
    float maxVelocity = 0.0f;//mm/s
    float maxAcceleration = 0.0f;//mm/s2
    
    AxisLimits(float minPosition, float maxPosition, float maxVelocity, float maxAcceleration) : minPosition(minPosition), maxPosition(maxPosition), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration) {}

    void SetMinPosition(float minPosition){
        this->minPosition = minPosition;
    }

    void SetMaxPosition(float maxPosition){
        this->maxPosition = maxPosition;
    }

    void SetMaxVelocity(float velocity){
        this->maxVelocity = velocity;
    }

    void SetMaxAcceleration(float acceleration){
        this->maxAcceleration = acceleration;
    }
};
