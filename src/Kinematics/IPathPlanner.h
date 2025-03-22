#pragma once

#include "../Axis/Axis.h"

class IPathPlanner {
public:
    virtual void AddAxis(Axis* axis) = 0;
    virtual Axis* GetAxis(uint8_t axisIndex) = 0;
    virtual uint8_t GetAxisCount() = 0;
    virtual void CalculateLimits(float feedrate) = 0;
    virtual void CalculateLimitsNoAccel(float feedrate) = 0;
    virtual bool Update() = 0;
    virtual bool UpdateNoAccel() = 0;

};
