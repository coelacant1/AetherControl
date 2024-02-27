#pragma once

#include "Axis.h"

class IPathPlanner {
public:
    enum Kinematics {
        Cartesian,
        CoreXY,
        CoreXZ,
        Delta,
        Idex,
        Polar
    };

    virtual void AddAxis(Axis* axis) = 0;
    virtual Axis* GetAxis(int axisIndex) = 0;
    virtual uint8_t GetAxisCount() = 0;
    virtual void CalculateLimits(float feedrate) = 0;
    virtual bool Update() = 0;

};