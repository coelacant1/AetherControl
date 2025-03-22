//handles list of gcode command objects
//iterates through list
#pragma once

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"
#include "../lib/ProtoTracer/Utils/Time/TimeStep.h"

#include <Arduino.h>
#include "IPathPlanner.h"

template<uint8_t axisCount>
class PathPlanner : public IPathPlanner {
private:
    Axis* axes[axisCount];
    float startPosition[axisCount];
    float endPosition[axisCount];
    uint8_t currentAxes = 0;
    elapsedMicros sinceUpdate;
    TimeStep tS = TimeStep(50);

    float velocity = 0.0f;
    float targetVelocity = 0.0f;
    float acceleration = 0.0f;
    float ratio = 0.0f;

    bool newCommand = true;

public:
    PathPlanner();

    void AddAxis(Axis* axis) override;
    Axis* GetAxis(uint8_t axisIndex) override;
    uint8_t GetAxisCount() override;
    void CalculateLimits(float feedrate) override;
    void CalculateLimitsNoAccel(float feedrate) override;
    bool Update() override;
    bool UpdateNoAccel() override;

};

#include "PathPlanner.tpp"
